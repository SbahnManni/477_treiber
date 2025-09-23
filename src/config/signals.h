#pragma once
//pcf8574_inputs
inline constexpr int SIFA_HAND = 0;
inline constexpr int SIFA_FUSS = 1;
inline constexpr int HAPUTSCHALTER_0 = 2;
inline constexpr int HAPUTSCHALTER_1 = 3;
inline constexpr int STROMABNEHMER_0 = 4;
inline constexpr int STROMABNEHMER_1 = 5;
inline constexpr int FAHRSCHALTER = 6;
inline constexpr int EP_LOESEN = 7;

// "analog_inputs": [
inline constexpr int HBL_DRUCK = 8;
inline constexpr int HL_DRUCK = 9;
inline constexpr int C_DRUCK = 10;
inline constexpr int HANDBREMSE = 11;

//  pcf8574_outputs
inline constexpr int LM_SIFA = 12;
inline constexpr int LM_HAUPTSCHALTER = 13;
inline constexpr int STOERLAMPE_1 = 14;
inline constexpr int STUERLAMPE_2 = 15;
inline constexpr int SIFA_SUMMER = 16;
inline constexpr int MV_EP_AN = 17;
inline constexpr int MV_EP_LOE = 18;
inline constexpr int KOMPRESSOR = 19;

//  "analog_outputs"
inline constexpr int A_TACHO = 20;
inline constexpr int A_FAHRSPANNUNG = 21;
inline constexpr int A_230V = 22;
inline constexpr int A_110V = 23;

//  "mpu9250"
inline constexpr int X = 24;
inline constexpr int Y = 25;
inline constexpr int Z = 26;