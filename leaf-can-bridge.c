#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#include "can-bridge-firmware.h"

// ================= BATTERY REPACKING CONFIGURATION =================
// Real Battery: CATL 51 kWh (96s, 6s×16 modules)
// BMS: AZEO 24 kWh (emulation for Leaf vehicle)
// Vehicle: Nissan Leaf expects 24 kWh
//
// Mapping:
// - Real pack voltage: 288-398.4V (3.0-4.15V per cell)
// - Real pack capacity: 51 kWh
// - Emulated capacity: 24 kWh (masked to vehicle)
// - Energy scaling factor: 24 / 51 = 0.47 (approximately)

#define REAL_KWH        51.0f   // CATL 51 kWh - actual battery capacity
#define EMU_KWH         24.0f   // AZEO 24 kWh - emulated for Leaf vehicle

// GID (Grid Index Data) conversion
// Real CATL 51 kWh: 51000 Wh / 80 Wh/GID = 637.5 GID (≈640)
// Emulated AZEO 24 kWh: 24000 Wh / 80 Wh/GID = 300 GID (≈312 with buffer)
#define GIDS_PACK_MAX_REAL   640    // Real CATL 51kWh max GID
#define GIDS_PACK_MAX        312    // Emulated AZEO 24kWh max GID for vehicle
#define SOC_MAX              1000

float soh = 1.00f;   // State of Health (new pack baseline)

// ================= LOOKUP TABLE (Voltage -> SOC) =================
typedef struct {
    float voltage;
    float soc;
} lut_entry_t;

// CATL 51 kWh BMS voltage curve (96s configuration, 4.15V - 3.0V range)
// Optimized for NCM cell chemistry with safe operating window
// Min: 96 cells * 3.0V = 288V (safe minimum, discharge limit)
// Max: 96 cells * 4.15V = 398.4V (safe maximum, charge limit)
// Total usable range: 110.4V window
lut_entry_t voltage_soc_lut[] = {
    {398.4f, 1.00f},  // 4.15V/cell - Maximum (100% SOC)
    {393.6f, 0.95f},  // 4.10V/cell
    {388.8f, 0.90f},  // 4.05V/cell
    {384.0f, 0.85f},  // 4.00V/cell
    {379.2f, 0.80f},  // 3.95V/cell
    {374.4f, 0.75f},  // 3.90V/cell
    {369.6f, 0.70f},  // 3.85V/cell
    {364.8f, 0.65f},  // 3.80V/cell
    {360.0f, 0.60f},  // 3.75V/cell
    {355.2f, 0.50f},  // 3.70V/cell
    {350.4f, 0.40f},  // 3.65V/cell
    {345.6f, 0.30f},  // 3.60V/cell
    {340.8f, 0.20f},  // 3.55V/cell
    {336.0f, 0.10f},  // 3.50V/cell - Low battery warning
    {330.0f, 0.05f},  // 3.44V/cell - Critical discharge
    {288.0f, 0.00f}   // 3.00V/cell - Minimum (0% SOC, safe discharge limit)
};

#define LUT_SIZE (sizeof(voltage_soc_lut)/sizeof(lut_entry_t))

// ================= GLOBALS =================
uint16_t LBC_MainGids;
uint16_t LBC_StateOfCharge;
uint16_t LBC_Voltage;

uint16_t emu_gids;
uint16_t emu_soc;

static float filtered_soc = 0.5f;
static int is_soc_initialized = 0;

// Track maximum seen energy for SOH calculation
static float max_seen_energy = 0.0f;

// ================= DIAGNOSTIC COUNTERS =================
// ADDED: Counters for message processing diagnostics
static uint32_t process_5BC_count = 0;   // Battery state message (0x5BC)
static uint32_t process_1DC_count = 0;   // Charge control message (0x1DC)
static uint32_t process_59E_count = 0;   // Battery capacity message (0x59E)

// ================= HELPERS =================
float interpolate_soc(float voltage) {
    for (int i = 0; i < (int)LUT_SIZE - 1; i++) {
        if (voltage <= voltage_soc_lut[i].voltage &&
            voltage >= voltage_soc_lut[i+1].voltage) {

            float v1 = voltage_soc_lut[i].voltage;
            float v2 = voltage_soc_lut[i+1].voltage;
            float s1 = voltage_soc_lut[i].soc;
            float s2 = voltage_soc_lut[i+1].soc;

            return s1 + (voltage - v1) * (s2 - s1) / (v2 - v1);
        }
    }

    if (voltage > voltage_soc_lut[0].voltage) return 1.0f;
    if (voltage < voltage_soc_lut[LUT_SIZE-1].voltage) return 0.0f;

    return 0.0f;
}

float smooth(float old, float new_val) {
    return (old * 0.8f + new_val * 0.2f);
}

float clamp01(float v) {
    if (v > 1.0f) return 1.0f;
    if (v < 0.0f) return 0.0f;
    return v;
}

float rate_limit(float old, float target, float max_up, float max_down) {
    float delta = target - old;
    if (delta > max_up) return old + max_up;
    if (delta < -max_down) return old - max_down;
    return target;
}

// ================= STATE OF HEALTH =================
void update_soh(float max_seen_energy) {
    float nominal = REAL_KWH;
    float estimated_soh = max_seen_energy / nominal;

    if (estimated_soh < 1.0f && estimated_soh > 0.7f) {
        soh = smooth(soh, estimated_soh);
    }
}

// ================= TEMPERATURE MAPPING =================
uint8_t map_temperature(uint8_t raw_temp) {
    if (raw_temp < 10) return 15;
    if (raw_temp > 50) return 45;
    return raw_temp;
}

// ================= VALIDATION =================
// CATL 51 kWh voltage validation (4.15V - 3.00V per cell)
// Min: 96 cells * 3.0V = 288V (safe discharge minimum)
// Max: 96 cells * 4.15V = 398.4V (safe charge maximum)
// Range: 270V to 410V (with safety margin for edge cases)
int validate_voltage(float voltage) {
    if (voltage < 270.0f || voltage > 410.0f) {
        fprintf(stderr, "ERROR: CATL 51kWh Voltage %.1f V out of valid range [270V-410V]\n", voltage);
        return 0;  // Invalid
    }
    return 1;  // Valid
}

// ================= MAIN PIPELINE (Message 0x5BC) =================
// This message contains battery state information from BMS
// We read the real CATL 51kWh data and convert it to AZEO 24kWh for vehicle
void process_5BC(uint8_t *data) {
    if (data == NULL) return;

    // Increment diagnostic counter
    process_5BC_count++;

    // Decode original GIDs and SOC from CATL BMS (10-bit format)
    LBC_MainGids = ((data[0] << 2) | (data[1] >> 6));      // 0-640 GID range
    LBC_StateOfCharge = ((data[4] << 2) | (data[5] >> 6)); // 0-1000 SOC range

    // Decode voltage (0.1V per bit)
    LBC_Voltage = (data[2] << 8) | data[3];
    float voltage = (float)LBC_Voltage * 0.1f;

    // Validate voltage from CATL BMS
    if (!validate_voltage(voltage)) {
        return; // Ignore invalid data
    }

    // Calculate SOC from three sources
    float soc_from_voltage = interpolate_soc(voltage);
    float soc_from_lbc = clamp01((float)LBC_StateOfCharge / (float)SOC_MAX);
    float soc_from_gids = clamp01((float)LBC_MainGids / (float)GIDS_PACK_MAX_REAL);

    // Hybrid SOC calculation (weighted average for stability)
    float soc_hybrid = (
        soc_from_lbc * 0.40f +
        soc_from_gids * 0.40f +
        soc_from_voltage * 0.20f
    );

    // Filter and apply rate limits (suppress voltage sag under load)
    if (!is_soc_initialized) {
        filtered_soc = soc_hybrid;
        is_soc_initialized = 1;
    } else {
        float smoothed = smooth(filtered_soc, soc_hybrid);
        // Rate limits: 0.006 up (fast charge), 0.002 down (slow discharge display)
        filtered_soc = rate_limit(filtered_soc, smoothed, 0.006f, 0.002f);
    }
    filtered_soc = clamp01(filtered_soc);

    // =============== ENERGY SCALING (REPACKING LOGIC) ===============
    // Calculate real energy from actual CATL 51kWh battery
    float real_energy = filtered_soc * REAL_KWH * soh;  // 0-51 kWh
    
    // Track maximum energy for SOH calculation
    if (real_energy > max_seen_energy) {
        max_seen_energy = real_energy;
    }

    // Convert real energy to emulated AZEO 24kWh for vehicle
    // This automatically scales: real_energy / EMU_KWH
    // Example: 25.5 kWh real → 25.5 / 24 = 1.0625 SOC... clamped to 1.0
    // Or: 12 kWh real → 12 / 24 = 0.5 SOC
    if (EMU_KWH <= 0.0f) return;
    
    float emu_soc_f = real_energy / EMU_KWH;
    emu_soc_f = clamp01(emu_soc_f);

    // Convert SOC to GID/SOC values for vehicle (Nissan Leaf format)
    emu_soc  = (uint16_t)(emu_soc_f * SOC_MAX);        // 0-1000 SOC
    emu_gids = (uint16_t)(emu_soc_f * GIDS_PACK_MAX);  // 0-312 GID (scaled down)

    // Encode modified GIDs and SOC back to data buffer
    // Vehicle expects AZEO 24kWh format, so we use scaled down values
    data[0] = (emu_gids >> 2) & 0xFF;
    data[1] = (data[1] & 0x3F) | ((emu_gids & 0x03) << 6);

    // FIXED: Apply temperature mapping to ensure battery temp is correct
    // Temperature is typically in data[6] (some BMS variants)
    // Clamp temperature to safe range for vehicle reporting
    if (data[6] != 0) {
        data[6] = map_temperature(data[6]);
    }

    data[4] = (emu_soc >> 2) & 0xFF;
    data[5] = (data[5] & 0x3F) | ((emu_soc & 0x03) << 6);

    // ADDED: Diagnostic logging every 100 messages (~10 seconds at 10Hz)
    if (process_5BC_count % 100 == 0) {
        fprintf(stderr, "[0x5BC #%u] Real: %.1f kWh, GID=%d/%d, SOC=%d/1000, Temp=%dC, Voltage=%.1fV\n",
                process_5BC_count,
                real_energy,
                emu_gids,
                GIDS_PACK_MAX,
                emu_soc,
                data[6],
                voltage);
    }
}

// ================= CHARGE CONTROL (Message 0x1DC) =================
void process_1DC(uint8_t *data) {
    if (data == NULL) return;
    
    // Increment diagnostic counter
    process_1DC_count++;
    
    // Simple cap for max charging current (120A limit)
    if (data[2] > 120) {
        data[2] = 120;
    }

    // ADDED: Diagnostic logging for charge current
    if (process_1DC_count % 100 == 0) {
        fprintf(stderr, "[0x1DC #%u] Charge current limited: %dA\n",
                process_1DC_count,
                data[2]);
    }
}

// ================= BATTERY CAPACITY (Message 0x59E) =================
void process_59E(uint8_t *data) {
    if (data == NULL) return;
    
    // Increment diagnostic counter
    process_59E_count++;
    
    // Set battery capacity to AZEO 24kWh for Leaf vehicle
    // Format: 0.1 kWh per unit
    // AZEO 24 kWh = 240 units (0x00F0)
    // Do NOT report real 51 kWh capacity (0x01F8) as this would confuse vehicle
    data[0] = 0x00;   // High byte
    data[1] = 0xF0;   // Low byte (0x00F0 = 240 decimal = 24.0 kWh)
    // Note: Other bytes (data[2-7]) remain unchanged

    // ADDED: Diagnostic logging
    fprintf(stderr, "[0x59E #%u] Battery capacity set to 24.0 kWh (AZEO emulation)\n",
            process_59E_count);
}

// ================= CAN DISPATCHER =================
void can_bridge(uint32_t id, uint8_t *data) {
    if (data == NULL) return;

    switch (id) {
        case 0x5BC:
            process_5BC(data);
            break;
        case 0x1DC:
            process_1DC(data);
            break;
        case 0x59E:
            process_59E(data);
            break;
        default:
            break;
    }
}

// ================= BRIDGE GLUE =================
// CAN2 = CATL BMS Side (source of real battery data)
// CAN1 = Vehicle Side (receives masked/emulated data)
void can_handler(uint8_t source_can, CAN_FRAME *frame) {
    if (frame == NULL) return;

    uint8_t dest_can = (source_can == MYCAN1) ? MYCAN2 : MYCAN1;

    // Process BMS -> Vehicle traffic (only from CAN2 with real data)
    if (source_can == MYCAN2 && !frame->rtr && frame->dlc <= 8) {
        can_bridge(frame->ID, frame->data);  // Convert CATL 51kWh to AZEO 24kWh
    }

    PushCan(dest_can, CAN_TX, frame);
}

// ================= PERIODIC HOUSEKEEPING =================
void one_second_ping(void) {
    // Update State of Health based on maximum observed energy
    update_soh(max_seen_energy);
}