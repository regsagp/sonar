#pragma once

enum LedMode {
    LM_OFF = 0,
    LM_FLICK,
    LM_SLEEP,
    LM_LOW_BATT,
    LM_WIFI_INIT,
    LM_DOOR_CLOSED,
};

const char* LedModeStr(LedMode mode) {
    switch (mode)
    {
    case LedMode::LM_OFF:
        return "OFF";
    case LedMode::LM_SLEEP:
        return "SLEEP";
    case LedMode::LM_LOW_BATT:
        return "LOW_BATT";
    case LedMode::LM_WIFI_INIT:
        return "WIFI_INIT";
    case LedMode::LM_FLICK:
        return "FLICK";
    case LedMode::LM_DOOR_CLOSED:
        return "DOOR_CLOSED";
    
    default:
        return "UNKNOWN";
    }
}
void SetLedMode(LedMode mode);
