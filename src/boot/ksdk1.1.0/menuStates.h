//
// Created by Joseph Holland  on 11/01/2020.
//

#ifndef WARP_FIRMWARE_MENUSTATES_H
#define WARP_FIRMWARE_MENUSTATES_H

typedef enum {
    rawDecimal,
    rawHex,

    calibratedmg,
    calibratedHex,

    settingsTop,
    settingsTare,
    settingsCalibrate,

    settingsMovingAvg,

    settingsGain,

    settingsExit
} menuState;
#endif //WARP_FIRMWARE_MENUSTATES_H
