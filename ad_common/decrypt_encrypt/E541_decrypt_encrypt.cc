#include "E541_decrypt_encrypt.h"
#include <stdexcept>
#include <iostream>

namespace ad_common {
namespace crypto {
E541VinCrypto::E541VinCrypto() {
}

E541VinCrypto::~E541VinCrypto() {
}

std::string E541VinCrypto::encryptVin(const std::string& vin) {
    if (vin.empty()) {
        return "";
    }

    try {
        // 使用与数据包加密相同的固定IV
        std::vector<unsigned char> iv(FIXED_IV, FIXED_IV + 16);

        // 准备输入数据
        std::vector<unsigned char> plaintext(vin.begin(), vin.end());

        // 使用专门的VIN加密方法（使用标准填充）
        std::vector<unsigned char> vin_key(VIN_AES_KEY, VIN_AES_KEY + 16);
        std::vector<unsigned char> encrypted_data = aes128_encrypt_with_padding(plaintext, vin_key, iv);

        // 直接使用加密数据，不包含IV
        std::string encrypted_vin = hex_encode(encrypted_data);

        return encrypted_vin;

    } catch (const std::exception& e) {
        std::cerr << "[E541VinCrypto] Exception in encryptVin: " << e.what() << std::endl;
        return "";
    }
}

std::string E541VinCrypto::decryptVin(const std::string& encrypted_vin) {
    if (encrypted_vin.empty()) {
        return "";
    }

    try {
        // HEX解码
        std::vector<unsigned char> encrypted_data = hex_decode(encrypted_vin);

        if (encrypted_data.empty()) {
            std::cerr << "[E541VinCrypto] Invalid encrypted VIN data" << std::endl;
            return "";
        }

        // 直接使用固定的IV解密（使用标准填充）
        std::vector<unsigned char> iv(FIXED_IV, FIXED_IV + 16);
        std::vector<unsigned char> vin_key(VIN_AES_KEY, VIN_AES_KEY + 16);
        std::vector<unsigned char> decrypted_data = aes128_decrypt_with_padding(encrypted_data, vin_key, iv);

        // 转换为字符串
        std::string decrypted_vin(decrypted_data.begin(), decrypted_data.end());

        return decrypted_vin;

    } catch (const std::exception& e) {
        std::cerr << "[E541VinCrypto] Exception in decryptVin: " << e.what() << std::endl;
        return "";
    }
}

std::vector<unsigned char> E541VinCrypto::aes128_encrypt_with_padding(const std::vector<unsigned char>& plaintext,
                                                                       const std::vector<unsigned char>& key,
                                                                       const std::vector<unsigned char>& iv) {
    if (key.size() != AES_KEY_SIZE || iv.size() != AES_IV_SIZE) {
        throw std::runtime_error("AES key/iv size mismatch");
    }

    EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
    if (!ctx) {
        throw std::runtime_error("EVP_CIPHER_CTX_new failed");
    }

    if (EVP_EncryptInit_ex(ctx, EVP_aes_128_cbc(), nullptr, key.data(), iv.data()) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("EVP_EncryptInit_ex failed");
    }

    // 使用标准PKCS7填充（默认）
    std::vector<unsigned char> ciphertext(plaintext.size() + AES_KEY_SIZE);
    int outlen1 = 0, outlen2 = 0;

    if (EVP_EncryptUpdate(ctx, ciphertext.data(), &outlen1, plaintext.data(), (int)plaintext.size()) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("EVP_EncryptUpdate failed");
    }

    if (EVP_EncryptFinal_ex(ctx, ciphertext.data() + outlen1, &outlen2) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("EVP_EncryptFinal_ex failed");
    }

    ciphertext.resize(outlen1 + outlen2);
    EVP_CIPHER_CTX_free(ctx);
    return ciphertext;
}

std::vector<unsigned char> E541VinCrypto::aes128_decrypt_with_padding(const std::vector<unsigned char>& encrypted_data,
                                                                       const std::vector<unsigned char>& key,
                                                                       const std::vector<unsigned char>& iv) {
    if (key.size() != AES_KEY_SIZE || iv.size() != AES_IV_SIZE) {
        throw std::runtime_error("AES key/iv size mismatch");
    }

    EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
    if (!ctx) {
        throw std::runtime_error("EVP_CIPHER_CTX_new failed");
    }

    if (EVP_DecryptInit_ex(ctx, EVP_aes_128_cbc(), nullptr, key.data(), iv.data()) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("EVP_DecryptInit_ex failed");
    }

    // 使用标准PKCS7填充（默认）
    std::vector<unsigned char> plaintext(encrypted_data.size());
    int outlen1 = 0, outlen2 = 0;

    if (EVP_DecryptUpdate(ctx, plaintext.data(), &outlen1, encrypted_data.data(), (int)encrypted_data.size()) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("EVP_DecryptUpdate failed");
    }

    if (EVP_DecryptFinal_ex(ctx, plaintext.data() + outlen1, &outlen2) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("EVP_DecryptFinal_ex failed");
    }

    plaintext.resize(outlen1 + outlen2);
    EVP_CIPHER_CTX_free(ctx);
    return plaintext;
}

std::string E541VinCrypto::hex_encode(const std::vector<unsigned char>& data) {
    const char* hex_chars = "0123456789ABCDEF";
    std::string result;
    result.reserve(data.size() * 2);  // 预分配空间

    for (unsigned char byte : data) {
        result.push_back(hex_chars[(byte >> 4) & 0x0F]);  // 高4位
        result.push_back(hex_chars[byte & 0x0F]);         // 低4位
    }
    return result;
}

std::vector<unsigned char> E541VinCrypto::hex_decode(const std::string& hex_string) {
    std::vector<unsigned char> result;
    result.reserve(hex_string.length() / 2);

    for (size_t i = 0; i < hex_string.length(); i += 2) {
        if (i + 1 < hex_string.length()) {
            unsigned char byte = 0;
            char c1 = hex_string[i];
            char c2 = hex_string[i + 1];

            // 转换字符为数值
            if (c1 >= '0' && c1 <= '9') byte |= (c1 - '0') << 4;
            else if (c1 >= 'A' && c1 <= 'F') byte |= (c1 - 'A' + 10) << 4;
            else if (c1 >= 'a' && c1 <= 'f') byte |= (c1 - 'a' + 10) << 4;

            if (c2 >= '0' && c2 <= '9') byte |= (c2 - '0');
            else if (c2 >= 'A' && c2 <= 'F') byte |= (c2 - 'A' + 10);
            else if (c2 >= 'a' && c2 <= 'f') byte |= (c2 - 'a' + 10);

            result.push_back(byte);
        }
    }
    return result;
}

} // namespace crypto
} // namespace ad_common
