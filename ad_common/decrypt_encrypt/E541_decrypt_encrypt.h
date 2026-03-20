#pragma once

#include <string>
#include <vector>
#include <cstddef>
#include <openssl/evp.h>

namespace ad_common {
namespace crypto {


class E541VinCrypto {
public:
    E541VinCrypto();
    ~E541VinCrypto();
    E541VinCrypto(const E541VinCrypto&) = delete;
    E541VinCrypto& operator=(const E541VinCrypto&) = delete;

    std::string encryptVin(const std::string& vin);
    std::string decryptVin(const std::string& encrypted_vin);

private:
    // AES密钥和IV大小常量
    static constexpr size_t AES_KEY_SIZE = 16;
    static constexpr size_t AES_IV_SIZE = 16;

    // 固定的IV值（16字节）
    static constexpr unsigned char FIXED_IV[16] = {0xA7, 0x3E, 0x5C, 0x89, 0x1F, 0x4B, 0x2D, 0x60, 0xC8, 0xE1, 0xF9, 0xB0, 0x2A, 0x83, 0x4D, 0x1F};
    // VIN加密的固定AES密钥 (16字节 = 128位)
    static constexpr unsigned char VIN_AES_KEY[16] = {0x2a, 0x7b, 0x8c, 0x4d, 0x9e, 0x1f, 0x3a, 0x5b, 0x6c, 0x7d, 0x8e, 0x9f, 0x0a, 0x1b, 0x2c, 0x3d};

    /**
     * @brief AES-128-CBC 对称加密（使用标准填充）
     * @param plaintext 明文数据
     * @param key AES密钥
     * @param iv 初始化向量
     * @return 加密后的数据
     */
    std::vector<unsigned char> aes128_encrypt_with_padding(const std::vector<unsigned char>& plaintext,
                                                          const std::vector<unsigned char>& key,
                                                          const std::vector<unsigned char>& iv);

    /**
     * @brief AES-128-CBC 对称解密（使用标准填充）
     * @param encrypted_data 加密数据
     * @param key AES密钥
     * @param iv 初始化向量
     * @return 解密后的数据
     */
    std::vector<unsigned char> aes128_decrypt_with_padding(const std::vector<unsigned char>& encrypted_data,
                                                          const std::vector<unsigned char>& key,
                                                          const std::vector<unsigned char>& iv);

    /**
     * @brief HEX编码
     * @param data 二进制数据
     * @return HEX编码后的字符串
     */
    std::string hex_encode(const std::vector<unsigned char>& data);

    /**
     * @brief HEX解码
     * @param hex_string HEX编码的字符串
     * @return 解码后的二进制数据
     */
    std::vector<unsigned char> hex_decode(const std::string& hex_string);
};

} // namespace crypto
} // namespace ad_common