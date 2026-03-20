#include "ad_common/decrypt_encrypt/M32T_decrypt_encrypt.h"
#include <string>
#include <iostream>

int main() {
	std::string line;
	if(!std::getline(std::cin, line)) return 1;

	ad_common::crypto::M32tVinCrypto codec;
	std::string result = codec.decryptVin(line);
	std::cout << result << std::endl;

    result = codec.encryptVin(result);
    std::cout << result << std::endl;

	return 0;
}