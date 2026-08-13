#if defined(AP_ENABLE_CUSTOM_STORAGE) && AP_ENABLE_CUSTOM_STORAGE==1

#pragma once
#include <AP_HAL/AP_HAL.h>
#include <StorageManager/StorageManager.h>
#include <GCS_MAVLink/GCS.h>

// Constants for storage configuration
#define DATA_BUFFER_SIZE 127           ///< Size of data buffer (excluding null terminator)
#define CUSTOM_PARAM_UUID_LEN 36       ///< Standard UUID string length (36 chars for "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx")
#define CUSTOM_PARAM_PASS_LEN 32       ///< Maximum allowed password length
#define CUSTOM_PARAM_CRAFT_ID_LEN 16   ///< Maximum allowed craft ID length (e.g. "AA-TRT-101")
#define CUSTOM_PARAM_UIN_LEN 20        ///< Maximum allowed UIN length (fits remaining space in the 128-byte block)
#define CUSTOM_PARAM_DATA_SIZE 128     ///< Total allocated storage size in bytes

/**
 * @class AP_CustomStorage
 * @brief Custom parameter storage system for ArduPilot
 *
 * Provides persistent storage for custom parameters including:
 * - UUID (Universally Unique Identifier)
 * - Password/authentication tokens
 * - Craft ID (e.g. "AA-TRT-101")
 * - UIN (Unique Identification Number)
 *
 * Data is stored in flash memory with a header structure for validation.
 */
class AP_CustomStorage {
public:
    AP_CustomStorage();

    static AP_CustomStorage *get_singleton() { return _singleton; }

    bool _initialized = false;  ///< Tracks whether storage system has been initialized

    /**
     * @brief Initialize the storage system
     *
     * Verifies storage allocation and loads existing data from flash.
     * Initializes default header if none exists.
     */
    void init();

    /**
     * @brief Check initialization status
     * @return true if initialized, false otherwise
     */
    bool is_initialized();

    // Parameter operations

    /**
     * @brief Store a UUID string
     * @param uuid Null-terminated UUID string (must be 36 chars or less)
     * @return true if successful, false on error
     */
    bool set_uuid(const char *uuid);

    /**
     * @brief Retrieve stored UUID
     * @param buf Buffer to receive UUID string
     * @param len Length of provided buffer (must be > CUSTOM_PARAM_UUID_LEN)
     * @return true if successful, false on error
     */
    bool get_uuid(char *buf, uint8_t len) const;

    /**
     * @brief Store a password string
     * @param pass Null-terminated password string (32 chars or less)
     * @return true if successful, false on error
     */
    bool set_password(const char *pass);

    /**
     * @brief Retrieve stored password
     * @param buf Buffer to receive password string
     * @param len Length of provided buffer (must be > CUSTOM_PARAM_PASS_LEN)
     * @return true if successful, false on error
     */
    bool get_password(char *buf, uint8_t len) const;

    /**
     * @brief Store a craft ID string
     * @param craft_id Null-terminated craft ID string (must be 16 chars or less)
     * @return true if successful, false on error
     */
    bool set_craft_id(const char *craft_id);

    /**
     * @brief Retrieve stored craft ID
     * @param buf Buffer to receive craft ID string
     * @param len Length of provided buffer (must be > CUSTOM_PARAM_CRAFT_ID_LEN)
     * @return true if successful, false on error
     */
    bool get_craft_id(char *buf, uint8_t len) const;

    /**
     * @brief Store a UIN string
     * @param uin Null-terminated UIN string (must be 20 chars or less)
     * @return true if successful, false on error
     */
    bool set_uin(const char *uin);

    /**
     * @brief Retrieve stored UIN
     * @param buf Buffer to receive UIN string
     * @param len Length of provided buffer (must be > CUSTOM_PARAM_UIN_LEN)
     * @return true if successful, false on error
     */
    bool get_uin(char *buf, uint8_t len) const;

private:
    static AP_CustomStorage *_singleton;

    static StorageAccess _storage;  ///< Storage manager interface
    char _data[DATA_BUFFER_SIZE + 1];  ///< Data buffer (+1 for null terminator)

    /**
     * @brief Storage header structure
     *
     * Packed to ensure consistent layout in flash memory
     */
    struct PACKED StorageHeader {
        uint32_t magic;   ///< Magic number for format identification
        uint16_t version; ///< Format version number
    };

    // Storage configuration constants
    enum {
        MAX_STORAGE_SIZE = 128,    ///< Total available storage space
        HEADER_MAGIC = 0x52545343, ///< Magic number ('CSTR' in ASCII)
        HEADER_SIZE = sizeof(StorageHeader) ///< Size of header structure
    };

    /**
     * @brief Layout definition for parameter storage
     *
     * Defines the memory layout of parameters within the storage area
     */
    struct {
        const uint8_t uuid_offset = HEADER_SIZE;  ///< UUID storage offset from start
        const uint8_t uuid_length = CUSTOM_PARAM_UUID_LEN;  ///< Allocated UUID storage length
        const uint8_t pass_offset = CUSTOM_PARAM_UUID_LEN + HEADER_SIZE;  ///< Password storage offset
        const uint8_t pass_length = CUSTOM_PARAM_PASS_LEN;  ///< Allocated password storage length
        const uint8_t craft_id_offset = CUSTOM_PARAM_UUID_LEN + CUSTOM_PARAM_PASS_LEN + HEADER_SIZE;  ///< Craft ID storage offset
        const uint8_t craft_id_length = CUSTOM_PARAM_CRAFT_ID_LEN;  ///< Allocated craft ID storage length
        const uint8_t uin_offset = CUSTOM_PARAM_UUID_LEN + CUSTOM_PARAM_PASS_LEN + CUSTOM_PARAM_CRAFT_ID_LEN + HEADER_SIZE;  ///< UIN storage offset
        const uint8_t uin_length = CUSTOM_PARAM_UIN_LEN;  ///< Allocated UIN storage length
    } _layout;

    /**
     * @brief Get pointer to raw storage data
     * @return const char* Pointer to internal data buffer
     */
    const char *get_storage() const; // Declaration only!

    /**
     * @brief Set new storage data
     * @param new_string Data to store (will be truncated to DATA_BUFFER_SIZE)
     */
    void set_storage(const char *new_string);

    // Helper methods

    /**
     * @brief Load data from flash storage
     * @return true if successful, false on error
     */
    bool load_from_flash();

    /**
     * @brief Save current data to flash storage
     * @return true if successful, false on error
     */
    bool save_to_flash();
};

// Global instance declaration
extern AP_CustomStorage g_custom_storage;

#endif