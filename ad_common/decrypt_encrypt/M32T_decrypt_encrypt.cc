#include "M32T_decrypt_encrypt.h"
#include <cstring>
#include <algorithm>
#include <cstdio>

namespace ad_common {
namespace crypto {

M32tVinCrypto::M32tVinCrypto() {
	KeyExpansion(K_KEY_, round_keys_);
}

M32tVinCrypto::~M32tVinCrypto() {

}

// ========== AES 核心 ==========
inline unsigned char M32tVinCrypto::FFmul(unsigned char a, unsigned char b) {
	unsigned char bw[4]; unsigned char res=0;
	bw[0]=b; for(int i=1;i<4;i++){ bw[i]=bw[i-1]<<1; if(bw[i-1]&0x80) bw[i]^=0x1b; }
	for(int i=0;i<4;i++){ if((a>>i)&0x01) res^=bw[i]; }
	return res;
}

inline void M32tVinCrypto::SubBytes(unsigned char s[4][4]) {
	for(int r=0;r<4;r++) for(int c=0;c<4;c++) s[r][c]=SBOX_[s[r][c]];
}

inline void M32tVinCrypto::ShiftRows(unsigned char s[4][4]) {
	for(int r=1;r<4;r++){ unsigned char t[4]; for(int c=0;c<4;c++) t[c]=s[r][(c+r)%4]; for(int c=0;c<4;c++) s[r][c]=t[c]; }
}

inline void M32tVinCrypto::MixColumns(unsigned char s[4][4]) {
	for(int c=0;c<4;c++){ unsigned char t0=s[0][c],t1=s[1][c],t2=s[2][c],t3=s[3][c];
		s[0][c]=FFmul(0x02,t0)^FFmul(0x03,t1)^t2^t3;
		s[1][c]=t0^FFmul(0x02,t1)^FFmul(0x03,t2)^t3;
		s[2][c]=t0^t1^FFmul(0x02,t2)^FFmul(0x03,t3);
		s[3][c]=FFmul(0x03,t0)^t1^t2^FFmul(0x02,t3);
	}
}

inline void M32tVinCrypto::AddRoundKey(unsigned char s[4][4], unsigned char k[4][4]) {
	for(int c=0;c<4;c++) for(int r=0;r<4;r++) s[r][c]^=k[r][c];
}

void M32tVinCrypto::KeyExpansion(const unsigned char* key, unsigned char w[11][4][4]) {
	static const unsigned char rc[10]={0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x1b,0x36};
	for(int r=0;r<4;r++) for(int c=0;c<4;c++) w[0][r][c]=key[r+c*4];
	for(int i=1;i<=10;i++){
		for(int j=0;j<4;j++){
			unsigned char t[4];
			for(int r=0;r<4;r++) t[r]= j? w[i][r][j-1] : w[i-1][r][3];
			if(j==0){
				unsigned char temp=t[0];
				for(int r=0;r<3;r++) t[r]=SBOX_[t[(r+1)%4]];
				t[3]=SBOX_[temp];
				t[0]^=rc[i-1];
			}
			for(int r=0;r<4;r++) w[i][r][j]=w[i-1][r][j]^t[r];
		}
	}
}

void M32tVinCrypto::AES_Encrypt_Block(const unsigned char in[16], unsigned char out[16]) {
	unsigned char s[4][4];
	for(int r=0;r<4;r++) for(int c=0;c<4;c++) s[r][c]=in[c*4+r];
	AddRoundKey(s,round_keys_[0]);
	for(int i=1;i<=10;i++){ SubBytes(s); ShiftRows(s); if(i!=10) MixColumns(s); AddRoundKey(s,round_keys_[i]); }
	for(int r=0;r<4;r++) for(int c=0;c<4;c++) out[c*4+r]=s[r][c];
}

void M32tVinCrypto::OFB_Process(const unsigned char iv[16], const unsigned char* in, int len, unsigned char* out) {
	unsigned char prev[16]; std::memcpy(prev, iv, 16);
	int off=0;
	while(off<len){
		unsigned char ks[16]; AES_Encrypt_Block(prev, ks);
		int n = std::min(16, len - off);
		for(int i=0;i<n;i++) out[off+i] = in[off+i] ^ ks[i];
		std::memcpy(prev, ks, 16);
		off += n;
	}
}

// ========== 编解码工具 ==========
std::string M32tVinCrypto::bytes_to_hex(const unsigned char* data, int len) {
	std::string hex;
	for(int i=0;i<len;i++){
		char buf[3];
		std::snprintf(buf, sizeof(buf), "%02X", data[i]);
		hex += buf;
	}
	return hex;
}

bool M32tVinCrypto::hex_to_bytes(const std::string& hex, unsigned char* out, int out_len){
	if((int)hex.size()!=out_len*2) return false;
	auto hexchar_to_int = [](char c)->int{
		if(c>='0'&&c<='9') return c-'0';
		if(c>='A'&&c<='F') return c-'A'+10;
		if(c>='a'&&c<='f') return c-'a'+10;
		return -1;
	};
	for(int i=0;i<out_len;i++){
		int hi=hexchar_to_int(hex[2*i]), lo=hexchar_to_int(hex[2*i+1]);
		if(hi<0||lo<0) return false;
		out[i]=(unsigned char)((hi<<4)|lo);
	}
	return true;
}

std::string M32tVinCrypto::encryptVin(const std::string& input) {
	if(input.size() != 17) return "";
	std::string vin_prefix = input.substr(0, 4);
	std::string vin_suffix = input.substr(4, 13);
	unsigned char vin_plain[13]; std::memcpy(vin_plain, vin_suffix.c_str(), 13);
	unsigned char vin_cipher[13] = {0};
	OFB_Process(K_IV_, vin_plain, 13, vin_cipher);
	std::string cipher_hex = bytes_to_hex(vin_cipher, 13);
	return std::string(KEY_INDEX_) + vin_prefix + cipher_hex;
}

std::string M32tVinCrypto::decryptVin(const std::string& input) {
	if(input.size() < 32) return "";
	std::string vin_prefix = input.substr(2, 4);
	std::string vin_cipher_hex = input.substr(6, 26);
	unsigned char vin_cipher[13]={0}, vin_part[13]={0};
	if(!hex_to_bytes(vin_cipher_hex, vin_cipher, 13)) return "";
	OFB_Process(K_IV_, vin_cipher, 13, vin_part);
	return vin_prefix + std::string(reinterpret_cast<const char*>(vin_part), 13);
}

} // namespace crypto
} // namespace ad_common