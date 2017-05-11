// Data types for data to be transfered
// Need a struct with one variable and the packed attribute to make this an array in swift (in a simple way



// Battery
typedef struct  __attribute__((packed)) {
    float voltage;
    float temperature;
} BatteryDataType;


/// System information command response
/// @detail Contains the relevant information received with the system information command is already distributed into the follwing parameters. See the BM019 manual and the CR95HF manual for information on the the codes and flag meanings
/// @param resultCode 0x80 if no error
/// @param responseFlags
/// @param infoFlags bit 0 is set to 1 in case of error
/// @param errorCode error code
/// @param uid 8 bytes containing the uid, order already reversed, e.g. (colons only inserted for readability): E0:07:A0:00:00:0C:48:BD. All zeros in case of an error.
typedef struct  __attribute__((packed)) {
    uint8_t uid[8];
    uint8_t resultCode;
    uint8_t responseFlags;
    uint8_t infoFlags;
    uint8_t errorCode;
} SystemInformationDataType;


/// IDN command response, which is the device ID
/// @detail Contains the relevant information received with the IDN command is already distributed into the follwing parameters. See the BM019 manual and the CR95HF manual for information on the the codes and flag meanings
/// @param resultCode 0x00 if no error
/// @param deiceID 13 bytes containing the device ID.
typedef struct  __attribute__((packed)) {
    uint8_t resultCode;
    uint8_t deviceID[13];
    uint8_t romCRC[2];
} IDNDataType;


/// State of BM019 NFC module
typedef struct  __attribute__((packed)) {
    /// Indicates state of BM019 NFC module.
    /// @detail 0 ... NFC is not ready because ISO15693 protocol not (yet) set,
    /// @detail 1 ... NFC is ready because ISO15693 protocol is set.
    uint8_t nfcReady;
} NFCState;

/// The first 344 bytes of data as read from FRAM of the Freestyle Libre Sensor
/// @detail Contains 24 byes of header, 296 bytes of body with blood sugar data
/// and 24 bytes of footer
/// @param bytes 344 bytes containing the the raw data
typedef struct  __attribute__((packed)) {
    uint8_t allBytes[344];
} AllBytesDataType;

/// State of nfc reading
/// @Contains current and last values needed to track what blocks of date have to be read from the FreeStyle Libre via nfc
/// Nomenclature:
///    block ... block of 8 bytes of data of sensor FRAM. The smallest unit of data that can be read from the sensor
///    data  ... consecutive buffer of 6 bytes in sensor FRAM that comprises the FreeStyle Libre data for on point in time. It contains blood sugar data and temperature data and maybe some more information on sensor state.
///    index ... index (number of) the data
typedef struct  __attribute__((packed)) {
    uint8_t uid[8];                  // FreeStyle Libre serial number
    uint8_t oldUid[8];
    uint8_t nextTrend;               // number of next trend block to read
    uint8_t oldNextTrend;
    uint8_t nextHistory;             // number of next history block to read
    uint8_t oldNextHistory;
    uint16_t minutesSinceStart;      // minutes since start of sensor
    uint16_t oldMinutesSinceStart;
    byte fram[344];                  // buffer for Freestyle Libre FRAM data
    byte resultCodes[43];            // result codes from nfc read single block command
} Sensor;
