/******************************************************************************
 * Copyright 2023 ZF. All Rights Reserved.
 *****************************************************************************/

/**
 * @file
 * @brief Defines the CanFrame struct and CanClient interface.
 */

#ifndef ZF_UTIL_BYTE_H
#define ZF_UTIL_BYTE_H

#include <sstream>
#include <iomanip>

#include "zf_hal_can_driver/hal_can_driver_global.h"

/**
 * @namespace zf::driver::canbus
 */
BEGIN_NS_ZF_DRIVER_CANBUS

class Byte
{
public:
    template <typename T>
    static std::string Int2Hex(T val, size_t width = sizeof(T) * 2)
    {
        std::stringstream ss;
        ss << std::uppercase << std::setfill('0') << std::setw(width) << std::hex << (val | 0);
        return ss.str();
    }

    template <typename T>
    static std::string EcuFromCanId(T id)
    {
        if (id < 65536) // not greater than 0xFFFF
        {
            return "";
        }
        uint8_t high = static_cast<uint8_t>((id >> 16) & 0xFF); //& max value 0xFF for length 8, so limited length is 8
        if ((high & 0xF0) == 0xF0)                                  // Begin with 0xF
        {
            uint8_t srcId = id & 0xFF;
            return Byte::Int2Hex(srcId);//length: 2
        }
        else // Not begin with 0xF
        {
            std::string canId = Byte::Int2Hex(id);
            return canId.substr(canId.length() - 4);// length: 4: Extract the last four characters from the input string
        }
    }

    template <typename T>
    static std::string PGNFromCanId(T id)
    {
        // std::stringstream ssHex;
        // ssHex << std::uppercase << std::setfill('0') << std::setw(width) << std::hex << (id | 0);
        // if (ssHex.str().length() >= 6)
        // {
        //     return ssHex.str().substr(2, 4);
        // }
        // else
        // {
        //     return ssHex.str();
        // }
        uint8_t high;
        uint8_t low;
        std::string result = "";
        if (id >= 65536) // current PGN: Form 8 bits to 24 bits
        {
            high = static_cast<uint8_t>((id >> 16) & 0xFF); //& max value 0xFF for length 8, so limited length is 8
            low = static_cast<uint8_t>((id >> 8) & 0xFF);
            result += byte_to_hex(high);
            if ((high & 0xF0) != 0xF0)
            {
                result += "00";
            }
            else
            {
                result += byte_to_hex(low);
            }
        }
        else // not greater than 0xFFFF
        {
            std::stringstream ssHex;
            // ssHex << std::uppercase << std::setfill('0') << std::setw(width) << std::hex << (id | 0);
            ssHex << std::uppercase << std::setfill('0') << std::setw(4) << std::hex << (id | 0);
            return ssHex.str();
        }
        return result;
    }

    Byte();

    template <typename T>
    Byte(const T value) : m_value(static_cast<uint8_t>(value)) {}

    /**
     * @brief Constructor which takes a pointer to a one-byte unsigned integer.
     * @param value The pointer to a one-byte unsigned integer for construction.
     */
    explicit Byte(const uint8_t *value);

    /**
     * @brief Constructor which takes a reference to a one-byte unsigned integer.
     * @param value The reference to a one-byte unsigned integer for construction.
     */
    Byte(const Byte &value);

    /**
     * @brief Desctructor.
     */
    ~Byte() = default;

    /**
     * @brief Transform an integer with the size of one byte to its hexadecimal
     *        represented by a string.
     * @param value The target integer to transform.
     * @return Hexadecimal representing the target integer.
     */
    static std::string byte_to_hex(const uint8_t value);

    /**
     * @brief Transform an integer with the size of 4 bytes to its hexadecimal
     *        represented by a string.
     * @param value The target integer to transform.
     * @return Hexadecimal representing the target integer.
     */
    static std::string byte_to_hex(const uint32_t value);

    /**
     * @brief Transform an integer with the size of one byte to its binary
     *        represented by a string.
     * @param value The target integer to transform.
     * @return Binary representing the target integer.
     */
    static std::string byte_to_binary(const uint8_t value);

    /**
     * @brief Set the bit on a specified position to one.
     * @param pos The position of the bit to be set to one.
     */
    void set_bit_1(const int32_t pos);

    /**
     * @brief Set the bit on a specified position to zero.
     * @param pos The position of the bit to be set to zero.
     */
    void set_bit_0(const int32_t pos);

    /**
     * @brief Check if the bit on a specified position is one.
     * @param pos The position of the bit to check.
     * @return If the bit on a specified position is one.
     */
    bool is_bit_1(const int32_t pos) const;

    /**
     * @brief Reset this Byte by a specified one-byte unsigned integer.
     * @param value The one-byte unsigned integer to set this Byte.
     */
    void set_value(const uint8_t value);

    /**
     * @brief Reset the higher 4 bits as the higher 4 bits of a specified one-byte
     *        unsigned integer.
     * @param value The one-byte unsigned integer whose higher 4 bits are used to
     *        set this Byte's higher 4 bits.
     */
    void set_value_high_4_bits(const uint8_t value);

    /**
     * @brief Reset the lower 4 bits as the lower 4 bits of a specified one-byte
     *        unsigned integer.
     * @param value The one-byte unsigned integer whose lower 4 bits are used to
     *        set this Byte's lower 4 bits.
     */
    void set_value_low_4_bits(const uint8_t value);

    /**
     * @brief Reset some consecutive bits starting from a specified position with
     *        a certain length of another one-byte unsigned integer.
     * @param value The one-byte unsigned integer whose certain bits are used
     *        to set this Byte.
     * @param start_pos The starting position (from the lowest) of the bits.
     * @param length The length of the consecutive bits.
     */
    void set_value(const uint8_t value, const int32_t start_pos,
                   const int32_t length);

    /**
     * @brief Get the one-byte unsigned integer.
     * @return The one-byte unsigned integer.
     */
    uint8_t get_byte() const;

    /**
     * @brief Get a one-byte unsigned integer representing the higher 4 bits.
     * @return The one-byte unsigned integer representing the higher 4 bits.
     */
    uint8_t get_byte_high_4_bits() const;

    /**
     * @brief Get a one-byte unsigned integer representing the lower 4 bits.
     * @return The one-byte unsigned integer representing the lower 4 bits.
     */
    uint8_t get_byte_low_4_bits() const;

    /**
     * @brief Get a one-byte unsigned integer representing the consecutive bits
     *        from a specified position (from lowest) by a certain length.
     * @param start_pos The starting position (from lowest) of bits.
     * @param length The length of the selected consecutive bits.
     * @return The one-byte unsigned integer representing the selected bits.
     */
    uint8_t get_byte(const int32_t start_pos, const int32_t length) const;

    /**
     * @brief Transform to its hexadecimal represented by a string.
     * @return Hexadecimal representing the Byte.
     */
    std::string to_hex_string() const;

    /**
     * @brief Transform to its binary represented by a string.
     * @return Binary representing the Byte.
     */
    std::string to_binary_string() const;

private:
    uint8_t m_value;
};

END_NS_ZF_DRIVER_CANBUS

#endif // ZF_UTIL_BYTE_H
