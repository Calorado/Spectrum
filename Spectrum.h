/*
 * Spectrum Preprocessor v1.1.0
 * Copyright (c) 2022 Carlos de Diego
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef __SPECTRUM__

#define __SPECTRUM__

#include <cstdint>
#include <stdio.h> //size_t

namespace spectrum {

	//Encodes "size" bytes of data present in "input", and stores it in "output".
	//backendCompressor is a pointer to a function that compresses data with the 
	// backend algorithm that will be used. This is for checking whether 
	// some filters help or not. By default it uses a fast heuristic.
	//Returns the size of the encoded stream or -1 on failure.
	size_t spectrum_encode(const uint8_t* input, const size_t size, uint8_t* output,
		size_t(*backendCompressor)(const uint8_t*, const size_t) = nullptr);
	//Decodes contents in "encoded" to "decoded".
	//Returns 0 on success or -1 on failure or corrupted data.
	int spectrum_decode(const uint8_t* encoded, const size_t encodedSize, uint8_t* decoded, const size_t decodedSize);

	//For a given input size, returns a size for the output buffer that is big enough to
	// contain the processed stream.
	size_t spectrum_bound(const size_t size);
}

#ifdef SPECTRUM_IMPLEMENTATION

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <vector>

#if defined(_MSC_VER)
#define FORCE_INLINE __forceinline
#elif defined(__GNUC__) || defined(__clang__)
#define FORCE_INLINE inline __attribute__((always_inline))
#else
#define FORCE_INLINE inline
#endif

#if defined(__GNUC__) || defined(__clang__)
#define expect(expr,value)    (__builtin_expect ((expr),(value)) )
#define likely(expr)     expect((expr) != 0, 1)
#define unlikely(expr)   expect((expr) != 0, 0)
#else
#define likely(expr)     (expr)
#define unlikely(expr)   (expr)
#endif

//Probably not the correct way to do it but bleh
#if UINTPTR_MAX > UINT32_MAX
#define IS_64BIT 1
#else
#define IS_64BIT 0
#endif

#if defined(_MSC_VER)
#if defined(_M_AMD64)
#include <intrin.h>
#define x64
#endif
#elif defined(__GNUC__) || defined(__clang__)
#if defined(__amd64__)
#include <x86intrin.h>
#define x64
#endif
#endif

namespace spectrum {

	bool is_little_endian() {
		const union { uint16_t u; uint8_t c[2]; } LITTLE_ENDIAN_CHECK = { 1 };
		return LITTLE_ENDIAN_CHECK.c[0];
	}

#if IS_64BIT

	//Undefined behaviour if value == 0
	FORCE_INLINE uint64_t unsafe_int_log2(const uint64_t value) {
#if defined(_MSC_VER)
		unsigned long result;
		_BitScanReverse64(&result, value);
		return result;
#elif defined(__GNUC__) || defined(__clang__)
		return 63 - __builtin_clzll(value);
#else
		const uint8_t tab64[64] = {
			 0, 58,  1, 59, 47, 53,  2, 60,
			39, 48, 27, 54, 33, 42,  3, 61,
			51, 37, 40, 49, 18, 28, 20, 55,
			30, 34, 11, 43, 14, 22,  4, 62,
			57, 46, 52, 38, 26, 32, 41, 50,
			36, 17, 19, 29, 10, 13, 21, 56,
			45, 25, 31, 35, 16,  9, 12, 44,
			24, 15,  8, 23,  7,  6,  5, 63
		};

		uint64_t index = value;
		index |= index >> 1;
		index |= index >> 2;
		index |= index >> 4;
		index |= index >> 8;
		index |= index >> 16;
		index |= index >> 32;
		return tab64[index * 0x03f6eaf2cd271461 >> 58];
#endif
	}

	//Way faster than using log2(double), also returns 0 for a value of 0
	FORCE_INLINE uint64_t int_log2(const uint64_t value) {
#if defined(_MSC_VER) || defined(__GNUC__) || defined(__clang__)
		if (likely(value != 0))
			return unsafe_int_log2(value);
		return 0;
#endif  //Fallback already returns 0 when value == 0
		return unsafe_int_log2(value);
	}

	//Undefined behaviour when value == 0
	FORCE_INLINE uint64_t unsafe_bit_scan_forward(const uint64_t value) {
#if defined(_MSC_VER)
		unsigned long result;
		_BitScanForward64(&result, value);
		return result;
#elif defined(__GNUC__) || defined(__clang__)
		return __builtin_ctzll(value);
#else
		const unsigned int tab64[64] = {
			 0,  1,  2, 53,  3,  7, 54, 27,
			 4, 38, 41,  8, 34, 55, 48, 28,
			62,  5, 39, 46, 44, 42, 22,  9,
			24, 35, 59, 56, 49, 18, 29, 11,
			63, 52,  6, 26, 37, 40, 33, 47,
			61, 45, 43, 21, 23, 58, 17, 10,
			51, 25, 36, 32, 60, 20, 57, 16,
			50, 31, 19, 15, 30, 14, 13, 12
		};
		return tab64[(value & (0 - value)) * 0x022FDD63CC95386D >> 58];
#endif
	}

	//Returns the index of the first set bit, starting from the least significant, or 0 if the input is null
	FORCE_INLINE uint64_t bit_scan_forward(const uint64_t value) {
#if defined(_MSC_VER) || defined(__GNUC__) || defined(__clang__)
		if (likely(value != 0))
			return unsafe_bit_scan_forward(value);
		return 0;
#endif  //Fallback already returns 0 when value == 0
		return unsafe_bit_scan_forward(value);
	}

	struct FastIntHash {
		//Use top bits
		uint64_t operator()(const uint64_t value) {
			return value * 0xff51afd7ed558ccd;
		}
	};

#else  //32 bit functions

	FORCE_INLINE uint32_t unsafe_int_log2(uint32_t value) {
#if defined(_MSC_VER)
		unsigned long result;
		_BitScanReverse(&result, value);
		return result;
#elif defined(__GNUC__) || defined(__clang__)
		return 31 - __builtin_clz(value);
#else
		const uint8_t tab32[32] = {
			 0,  9,  1, 10, 13, 21,  2, 29,
			11, 14, 16, 18, 22, 25,  3, 30,
			 8, 12, 20, 28, 15, 17, 24,  7,
			19, 27, 23,  6, 26,  5,  4, 31
		};

		value |= value >> 1;
		value |= value >> 2;
		value |= value >> 4;
		value |= value >> 8;
		value |= value >> 16;
		return tab32[value * 0x07C4ACDDU >> 27];
#endif
	}

	FORCE_INLINE uint32_t int_log2(uint32_t value) {
#if defined(_MSC_VER) || defined(__GNUC__) || defined(__clang__)
		if (likely(value != 0))
			return unsafe_int_log2(value);
		return 0;
#endif
		return unsafe_int_log2(value);
	}

	FORCE_INLINE uint32_t unsafe_bit_scan_forward(uint32_t value) {
#if defined(_MSC_VER)
		unsigned long result;
		_BitScanForward(&result, value);
		return result;
#elif defined(__GNUC__) || defined(__clang__)
		return __builtin_ctz(value);
#else
		static uint8_t tab32[32] = {
			 0,  1, 28,  2, 29, 14, 24,  3,
			30, 22, 20, 15, 25, 17,  4,  8,
			31, 27, 13, 23, 21, 19, 16,  7,
			26, 12, 18,  6, 11,  5, 10,  9
		};
		return tab32[(value & (0 - value)) * 0x077CB531U >> 27];
#endif
	}

	FORCE_INLINE uint32_t bit_scan_forward(uint32_t value) {
#if defined(_MSC_VER) || defined(__GNUC__) || defined(__clang__)
		if (likely(value != 0))
			return unsafe_bit_scan_forward(value);
		return 0;
#endif
		return unsafe_bit_scan_forward(value);
	}

	struct FastIntHash {
		uint32_t operator()(const uint32_t value) {
			return value * 0x27d4eb2d;
		}
	};

#endif

	FORCE_INLINE uint64_t read_uint64le(const uint8_t* const ptr) {
		if (is_little_endian()) {
			uint64_t value;
			memcpy(&value, ptr, 8);
			return value;
		}
		uint64_t value = 0;
		for (int i = 0; i < 8; i++)
			value |= (uint64_t)ptr[i] << i * 8;
		return value;
	}
	FORCE_INLINE void write_uint64le(uint8_t* const ptr, const uint64_t value) {
		if (is_little_endian())
			memcpy(ptr, &value, 8);
		else {
			for (int i = 0; i < 8; i++)
				ptr[i] = value >> i * 8;
		}
	}
	FORCE_INLINE uint32_t read_uint32le(const uint8_t* const ptr) {
		if (is_little_endian()) {
			uint32_t value;
			memcpy(&value, ptr, 4);
			return value;
		}
		uint32_t value = 0;
		for (int i = 0; i < 4; i++)
			value |= (uint64_t)ptr[i] << i * 8;
		return value;
	}
	FORCE_INLINE void write_uint32le(uint8_t* const ptr, const uint32_t value) {
		if (is_little_endian())
			memcpy(ptr, &value, 4);
		else {
			for (int i = 0; i < 4; i++)
				ptr[i] = value >> i * 8;
		}
	}
	FORCE_INLINE uint16_t read_uint16le(const uint8_t* const ptr) {
		uint16_t value;
		if (is_little_endian())
			memcpy(&value, ptr, 2);
		else
			value = ptr[0] | (ptr[1] << 8);
		return value;
	}
	FORCE_INLINE void write_uint16le(uint8_t* const ptr, const uint16_t value) {
		if (is_little_endian())
			memcpy(ptr, &value, 2);
		else {
			for (int i = 0; i < 2; i++)
				ptr[i] = value >> i * 8;
		}
	}

	//Encodes the value, and advances the pointer
	void write_LEB128(uint8_t*& out, size_t value) {
		do {
			uint8_t byte = value & 0x7F;
			value >>= 7;
			byte |= (value > 0) << 7;
			*out++ = byte;
		} while (value);
	}
	//Decodes a value, advances the pointer, and determines whether an out of bounds or overflow has ocurred
	int read_LEB128(const uint8_t*& in, const uint8_t* const inEnd, size_t* value) {
		*value = 0;
		uint8_t byte;
		size_t iterations = 0;
		do {
			if (in == inEnd || iterations >= (IS_64BIT ? 10 : 5))
				return -1;
			byte = *in++;
			*value |= (size_t)(byte & 0x7F) << iterations * 7;
			iterations++;
		} while (byte & 0x80);
		return 0;
	}

	//A hash table which does not check for collisions
	template<class Value, class Hash>
	class HashTable {
		Value* arr = nullptr;
		int hashShift;
	public:
		//Use 2^x sizes to avoid the use of modulo operator
		HashTable() {}
		~HashTable() {
			delete[] arr;
		}
		void init(const int logSize) {
			arr = new Value[(size_t)1 << logSize]();
			hashShift = (IS_64BIT ? 64 : 32) - logSize;
		}
		Value& operator[](const size_t value) {
			return arr[Hash{}(value) >> hashShift];
		}
		void reset() {
			std::fill_n(arr, (size_t)1 << ((IS_64BIT ? 64 : 32) - hashShift), 0);
		}
	};

	//https://github.com/romeric/fastapprox/blob/master/fastapprox/src/fastlog.h
	FORCE_INLINE float fast_log2(float x) {
		union { float f; uint32_t i; } vx = { x };
		union { uint32_t i; float f; } mx = { (vx.i & 0x007FFFFF) | 0x3f000000 };
		float y = vx.i;
		y *= 1.1920928955078125e-7f;

		return y - 124.22551499f
			- 1.498030302f * mx.f
			- 1.72587999f / (0.3520887068f + mx.f);
	}

	float calculate_entropy(const uint8_t* buf, const size_t bufsize) {

		uint32_t hist[2048];
		std::fill(hist, hist + 256 * 8, 0);
		size_t pos = 0;
		const size_t fastLoopEnd = bufsize & ~0x7;
		for (; pos < fastLoopEnd; pos += 8) {
			if (IS_64BIT) {
				uint64_t w;
				memcpy(&w, buf + pos, 8);
				hist[0 + (w >> 0 & 0xFF)]++;
				hist[256 + (w >> 8 & 0xFF)]++;
				hist[512 + (w >> 16 & 0xFF)]++;
				hist[768 + (w >> 24 & 0xFF)]++;
				hist[1024 + (w >> 32 & 0xFF)]++;
				hist[1280 + (w >> 40 & 0xFF)]++;
				hist[1536 + (w >> 48 & 0xFF)]++;
				hist[1792 + (w >> 56 & 0xFF)]++;
			}
			else {
				uint32_t w;
				memcpy(&w, buf + pos, 4);
				hist[0 + (w >> 0 & 0xFF)]++;
				hist[256 + (w >> 8 & 0xFF)]++;
				hist[512 + (w >> 16 & 0xFF)]++;
				hist[768 + (w >> 24 & 0xFF)]++;
				memcpy(&w, buf + pos + 4, 4);
				hist[1024 + (w >> 0 & 0xFF)]++;
				hist[1280 + (w >> 8 & 0xFF)]++;
				hist[1536 + (w >> 16 & 0xFF)]++;
				hist[1792 + (w >> 24 & 0xFF)]++;
			}
		}
		for (; pos < bufsize; pos++)
			hist[256 * (pos & 0x7) + buf[pos]]++;

		const float probDiv = 1 / float(bufsize);
		float entropy = 0;

		for (size_t b = 0; b < 256; b++) {

			size_t count = hist[0 + b] + hist[256 + b] + hist[512 + b] + hist[768 + b] +
				hist[1024 + b] + hist[1280 + b] + hist[1536 + b] + hist[1792 + b];

			if (count) {
				const float prob = count * probDiv;
				entropy += -(prob * fast_log2(prob));
			}
		}
		return entropy;
	}

	const int CALL_ADDRESS = 0;
	const int JUMP_ADDRESS = 1;
	const int MOV_ADDRESS = 2;
	const int LEA_ADDRESS = 3;

	size_t x86_encode(const uint8_t* input, const size_t size, uint8_t* output) {

		const uint8_t* const start = input;
		//Allows to read up to 4 bytes of opcode and 4 bytes of address
		const uint8_t* const end = size < 7 ? input : input + size - 7;

		uint32_t* addressBuf = nullptr;
		uint8_t* addressTypeBuf = nullptr;
		uint8_t* bitBuf = nullptr;
		HashTable<uint8_t, FastIntHash> bcjTable;
		try {
			//Each instruction takes 1 byte for the opcode and 4 for the offset, so it is impossible to have more than NUMBER_BYTES / 5 instructions
			addressBuf = new uint32_t[size / 5 + 1];
			addressTypeBuf = new uint8_t[size / 5 + 1];
			bitBuf = new uint8_t[size / 8 + 1]();
			bcjTable.init(20);
		}
		catch (std::bad_alloc& e) {
			delete[] addressBuf;
			delete[] addressTypeBuf;
			delete[] bitBuf;
			return -1;
		}

		uint32_t* addressIt = addressBuf;
		uint8_t* addressTypeIt = addressTypeBuf;
		size_t bitCount = 0;

		size_t callAddressCount = 0;
		size_t jumpAddressCount = 0;
		size_t movAddressCount = 0;
		size_t leaAddressCount = 0;

		uint8_t* mainIt = output;

		for (; input < end; ) {

			//Perform two passes: first one is to know which addresses are repeated so that we dont have false E8 and E9.
			//I have decided to do this in blocks. Very large ones could start getting more collisions, and thus reduce compression.
			const size_t BLOCK_SIZE = (1 << 22);
			const uint8_t* nextCheckpoint = end - input < BLOCK_SIZE + BLOCK_SIZE / 4 ? end : input + BLOCK_SIZE;
			bcjTable.reset();

			for (const uint8_t* it = input; it < nextCheckpoint; ) {
				const uint32_t opcode = read_uint16le(it);

				if ((opcode & 0xFE) == 0xE8) {
					it++;
					uint32_t address = read_uint32le(it);
					address += (it - start);

					uint8_t* entry = &bcjTable[address];
					*entry += (*entry < 2);
					it += 4;
				}
				else if ((opcode & 0xF0FF) == 0x800F) {
					it += 2;
					uint32_t address = read_uint32le(it);
					address += (it - start);

					uint8_t* entry = &bcjTable[address];
					*entry += (*entry < 2);
					it += 4;
				}
				else {
					it++;
				}
			}

			for (; input < nextCheckpoint; ) {

				const uint32_t opcode = read_uint32le(input);

				if ((opcode & 0xFF) == 0xE8) { //Call
					*mainIt++ = 0xE8;
					input++;

					uint32_t relative = read_uint32le(input);
					uint32_t address = relative + (input - start);

					//If the relative offset is very small, then it probably is a CALL instruction
					if (bcjTable[address] >= 2 || std::abs(int(relative)) < 0x8000) {
						bitBuf[bitCount / 8] |= 1 << bitCount % 8;
						*addressIt++ = address;
						*addressTypeIt++ = CALL_ADDRESS;
						callAddressCount++;
						input += 4;
					}
					bitCount++;
				}
				else if ((opcode & 0xFF) == 0xE9) { //Jump
					*mainIt++ = 0xE9;
					input++;

					uint32_t relative = read_uint32le(input);
					uint32_t address = relative + (input - start);

					//If the relative offset is very small, then it probably is a JUMP instruction
					if (bcjTable[address] >= 2 || std::abs(int(relative)) < 0x8000) {
						bitBuf[bitCount / 8] |= 1 << bitCount % 8;
						*addressIt++ = address;
						*addressTypeIt++ = JUMP_ADDRESS;
						jumpAddressCount++;
						input += 4;
					}
					bitCount++;
				}
				else if ((opcode & 0xF0FF) == 0x800F) { //Branch
					write_uint16le(mainIt, opcode & 0xFFFF);
					mainIt += 2;
					input += 2;

					uint32_t address = read_uint32le(input);
					address += (input - start);

					*addressIt++ = address;
					*addressTypeIt++ = JUMP_ADDRESS;
					jumpAddressCount++;
					input += 4;
				}
				else if ((opcode & 0xC7FD) == 0x0589) { //MOV reg,[addr], mov [addr],reg
					write_uint16le(mainIt, opcode & 0xFFFF);
					mainIt += 2;
					input += 2;

					uint32_t address = read_uint32le(input);
					address += (input - start);

					*addressIt++ = address;
					*addressTypeIt++ = MOV_ADDRESS;
					movAddressCount++;
					input += 4;
				}
				else if ((opcode & 0xC7FFFB) == 0x058D48) { //REX + LEA
					write_uint32le(mainIt, opcode & 0xFFFFFF);
					mainIt += 3;
					input += 3;

					uint32_t address = read_uint32le(input);
					address += (input - start);

					*addressIt++ = address;
					*addressTypeIt++ = LEA_ADDRESS;
					leaAddressCount++;
					input += 4;
				}
				else {
					*mainIt++ = opcode & 0xFF;
					input++;
				}
			}
		}

		while (input < start + size)
			*mainIt++ = *input++;

		size_t mainSize = mainIt - output;
		size_t metadataBytes = 0;
		metadataBytes += 1 + int_log2(callAddressCount) / 7;
		metadataBytes += 1 + int_log2(jumpAddressCount) / 7;
		metadataBytes += 1 + int_log2(movAddressCount) / 7;
		metadataBytes += 1 + int_log2(leaAddressCount) / 7;
		metadataBytes += 1 + int_log2((bitCount + 7) / 8) / 7;
		size_t bitSize = (bitCount + 7) / 8;

		uint8_t* callBuf = output + metadataBytes;
		uint8_t* jumpBuf = callBuf + callAddressCount * 4;
		uint8_t* movBuf = jumpBuf + jumpAddressCount * 4;
		uint8_t* leaBuf = movBuf + movAddressCount * 4;
		memmove(leaBuf + leaAddressCount * 4 + bitSize, output, mainSize);
		memcpy(leaBuf + leaAddressCount * 4, bitBuf, bitSize);
		size_t totalSize = metadataBytes + size + bitSize;

		const size_t totalAddressCount = callAddressCount + jumpAddressCount + movAddressCount + leaAddressCount;
		for (size_t i = 0; i < totalAddressCount; i++) {
			switch (addressTypeBuf[i]) {
			case CALL_ADDRESS:
				write_uint32le(callBuf, addressBuf[i]);
				callBuf += 4;
				break;
			case JUMP_ADDRESS:
				write_uint32le(jumpBuf, addressBuf[i]);
				jumpBuf += 4;
				break;
			case MOV_ADDRESS:
				write_uint32le(movBuf, addressBuf[i]);
				movBuf += 4;
				break;
			case LEA_ADDRESS:
				write_uint32le(leaBuf, addressBuf[i]);
				leaBuf += 4;
				break;
			}
		}

		write_LEB128(output, callAddressCount);
		write_LEB128(output, jumpAddressCount);
		write_LEB128(output, movAddressCount);
		write_LEB128(output, leaAddressCount);
		write_LEB128(output, bitSize);

		delete[] addressBuf;
		delete[] addressTypeBuf;
		delete[] bitBuf;

		return totalSize;
	}

	size_t x86_decode(const uint8_t* encoded, const uint8_t* const encodedEnd, const size_t size, uint8_t* decoded) {

		const uint8_t* const metadataStart = encoded;

		size_t callSize, jumpSize, movSize, leaSize, bitSize;
		if (read_LEB128(encoded, encodedEnd, &callSize))
			return -1;
		if (read_LEB128(encoded, encodedEnd, &jumpSize))
			return -1;
		if (read_LEB128(encoded, encodedEnd, &movSize))
			return -1;
		if (read_LEB128(encoded, encodedEnd, &leaSize))
			return -1;
		if (read_LEB128(encoded, encodedEnd, &bitSize))
			return -1;

		//Make sure all sizes are within encoded stream size. 
		//Better go one by one to ensure we do not get overflows.
		if (callSize * 4 > encodedEnd - encoded)
			return -1;
		if (jumpSize * 4 > encodedEnd - encoded - callSize * 4)
			return -1;
		if (movSize * 4 > encodedEnd - encoded - callSize * 4 - jumpSize * 4)
			return -1;
		if (leaSize * 4 > encodedEnd - encoded - callSize * 4 - jumpSize * 4 - movSize * 4)
			return -1;
		if (bitSize > encodedEnd - encoded - callSize * 4 - jumpSize * 4 - movSize * 4 - leaSize * 4)
			return -1;
		//There must be at least 4 bytes for main to be able to read the opcode OR the stream is very small
		if (encodedEnd - encoded - callSize * 4 - jumpSize * 4 - movSize * 4 - leaSize * 4 - bitSize < std::min((size_t)(encodedEnd - encoded), (size_t)4))
			return -1;

		const size_t metadataSize = encoded - metadataStart;

		const uint8_t* callIt = encoded;
		const uint8_t* jumpIt = callIt + callSize * 4;
		const uint8_t* movIt = jumpIt + jumpSize * 4;
		const uint8_t* leaIt = movIt + movSize * 4;
		const uint8_t* bitBuf = leaIt + leaSize * 4;
		const uint8_t* mainIt = bitBuf + bitSize;
		size_t bitCount = 0;

		const uint8_t* const start = decoded;
		const uint8_t* end = size < 7 ? decoded : decoded + size - 7;

		for (; decoded < end;) {

			//On a correct stream we will always have more than 4 bytes to read
			if (encodedEnd - mainIt < 4)
				return -1;
			const uint32_t opcode = read_uint32le(mainIt);

			if ((opcode & 0xFF) == 0xE8) {
				*decoded++ = 0xE8;
				mainIt++;

				//bitBuf always starts behind mainIt, and it will advance slower, 
				// so if mainIt is not out of bounds, neither is bitBuf.
				if (bitBuf[bitCount / 8] & (1 << bitCount % 8)) {
					//Would read from next buffer, but not from out of bounds
					if (jumpIt - callIt < 4)
						return -1;
					uint32_t address = read_uint32le(callIt);
					callIt += 4;

					address -= (decoded - start);
					memcpy(decoded, &address, 4);
					decoded += 4;
				}
				bitCount++;
			}
			else if ((opcode & 0xFF) == 0xE9) {
				*decoded++ = 0xE9;
				mainIt++;

				if (bitBuf[bitCount / 8] & (1 << bitCount % 8)) {
					if (movIt - jumpIt < 4)
						return -1;
					uint32_t address = read_uint32le(jumpIt);
					jumpIt += 4;

					address -= (decoded - start);
					memcpy(decoded, &address, 4);
					decoded += 4;
				}
				bitCount++;
			}
			else if ((opcode & 0xF0FF) == 0x800F) {
				write_uint16le(decoded, opcode & 0xFFFF);
				decoded += 2;
				mainIt += 2;

				if (movIt - jumpIt < 4)
					return -1;
				uint32_t address = read_uint32le(jumpIt);
				jumpIt += 4;

				address -= (decoded - start);
				memcpy(decoded, &address, 4);
				decoded += 4;
			}
			else if ((opcode & 0xC7FD) == 0x0589) {
				write_uint16le(decoded, opcode & 0xFFFF);
				decoded += 2;
				mainIt += 2;

				if (leaIt - movIt < 4)
					return -1;
				uint32_t address = read_uint32le(movIt);
				movIt += 4;

				address -= (decoded - start);
				memcpy(decoded, &address, 4);
				decoded += 4;
			}
			else if ((opcode & 0xC7FFFB) == 0x058D48) {
				write_uint32le(decoded, opcode & 0xFFFFFF);
				decoded += 3;
				mainIt += 3;

				if (bitBuf - leaIt < 4)
					return -1;
				uint32_t address = read_uint32le(leaIt);
				leaIt += 4;

				address -= (decoded - start);
				memcpy(decoded, &address, 4);
				decoded += 4;
			}
			else {
				*decoded++ = opcode & 0xFF;
				mainIt++;
			}
		}

		if (decoded < start + size) {
			//Insufficient bytes to end
			if (encodedEnd - mainIt < start + size - decoded)
				return -1;
			while (decoded < start + size)
				*decoded++ = *mainIt++;
		}

		return size + bitSize + metadataSize;
	}

	size_t delta_encode(const uint8_t* input, const size_t size, uint8_t* output, const size_t channels) {

		const uint8_t* const end = input + size;

		const size_t minBuf = std::min(size, channels);
		memcpy(output, input, minBuf);
		input += minBuf;
		output += minBuf;

#ifdef x64
		for (; input + 15 < end;) {
			__m128i cur = _mm_loadu_si128((__m128i*)input);
			__m128i prev = _mm_loadu_si128((__m128i*)(input - channels));
			cur = _mm_sub_epi8(cur, prev);
			_mm_storeu_si128((__m128i*)output, cur);
			input += 16;
			output += 16;
		}
#endif
		for (; input < end; input++)
			*output++ = *input - *(input - channels);

		return size;
	}

	size_t delta_decode(const uint8_t* input, const size_t size, uint8_t* output, const size_t channels) {

		const uint8_t* const end = input + size;

		const size_t minBuf = std::min(size, channels);
		memcpy(output, input, minBuf);
		input += minBuf;
		output += minBuf;

		for (; input < end; input++)
			*output++ = *input + *(output - channels);

		return size;
	}

	size_t lpc_encode(const uint8_t* input, const size_t size, uint8_t* output, const size_t channels) {

		const uint8_t* const end = input + size;

		const size_t minBuf = std::min(size, channels * 2);
		memcpy(output, input, minBuf);
		input += minBuf;
		output += minBuf;

#ifdef x64
		for (; input + 15 < end;) {
			__m128i cur = _mm_loadu_si128((__m128i*)input);
			__m128i prev1 = _mm_loadu_si128((__m128i*)(input - channels));
			__m128i prev2 = _mm_loadu_si128((__m128i*)(input - channels * 2));
			cur = _mm_add_epi8(_mm_sub_epi8(_mm_sub_epi8(cur, prev1), prev1), prev2);
			_mm_storeu_si128((__m128i*)output, cur);
			input += 16;
			output += 16;
		}
#endif
		for (; input < end; input++)
			*output++ = *input - *(input - channels) * 2 + *(input - channels * 2);

		return size;
	}

	size_t lpc_decode(const uint8_t* input, const size_t size, uint8_t* output, const size_t channels) {

		const uint8_t* const end = input + size;

		const size_t minBuf = std::min(size, channels * 2);
		memcpy(output, input, minBuf);
		input += minBuf;
		output += minBuf;

		for (; input < end; input++)
			*output++ = *input + *(output - channels) * 2 - *(output - channels * 2);

		return size;
	}

	const int NONE_FILTER = 0;
	const int X86_FILTER = 1;
	const int DELTA_FILTER = 2;
	const int LPC_FILTER = 3;

	struct PreprocessorBlock {
		size_t filter;
		size_t channels;
		size_t size;
	};

	size_t spectrum_encode(const uint8_t* input, const size_t size, uint8_t* output, size_t(*backendCompressor)(const uint8_t*, const size_t)) {

		const size_t PRE_BLOCK_SIZE = 64 * 1024;

		//Buffer used to store delta transform, and determine if it helps compression
		uint8_t* deltaBuf = nullptr;
		std::vector<PreprocessorBlock> filters;
		try {
			filters.reserve(size / PRE_BLOCK_SIZE + 1);
			filters.push_back({ 255, 0, 0 });
			deltaBuf = new uint8_t[PRE_BLOCK_SIZE];
		}
		catch (std::bad_alloc& e) {
			return -1;
		}

		const uint8_t* const inputStart = input;
		const uint8_t* const inputEnd = input + size;

		for (; input < inputEnd; ) {

			const size_t thisBlockSize = std::min((size_t)(inputEnd - input), PRE_BLOCK_SIZE);
			//if the block is small is probably better to just use the same filter,
			//as we could have more false positives without enough data
			if (thisBlockSize < 4096) {
				if (filters.size() > 1)
					filters.back().size += thisBlockSize;
				else
					filters.push_back({ NONE_FILTER, 0, thisBlockSize });
				input += thisBlockSize;
				continue;
			}
			const uint8_t* const thisBlockStart = input;
			const uint8_t* const thisBlockEnd = input + thisBlockSize;

			//x86 detection
			size_t e8count = 0;
			size_t duplicates = 0;

			HashTable<uint32_t, FastIntHash> jumpTable;
			jumpTable.init(12);
			const uint8_t* iterator = thisBlockStart;

			const size_t niceDuplicates = 128 * thisBlockSize / PRE_BLOCK_SIZE;
			const size_t maxE8Count = 1024 * thisBlockSize / PRE_BLOCK_SIZE;
#ifdef x64
			const uint8_t* end = thisBlockEnd - 19;
			const __m128i e8 = _mm_set1_epi8(0xE8);
#else
			const uint8_t* end = thisBlockEnd - 4;
#endif
			for (; iterator < end; ) {

#ifdef x64
				__m128i bytes = _mm_loadu_si128((__m128i*)iterator);
				bytes = _mm_cmpeq_epi8(bytes, e8);
				const size_t mask = _mm_movemask_epi8(bytes);

				//No byte is e8, skip forward
				if (mask == 0) {
					iterator += 16;
					continue;
				}

				//Go to the index of the first e8 found
				const size_t index = unsafe_bit_scan_forward(mask);
				iterator += index + 1;
#else
				const uint8_t opcode = *iterator++;
				if (opcode != 0xE8)
					continue;
#endif

				uint32_t offset;
				memcpy(&offset, iterator, 4);
				offset += reinterpret_cast<size_t>(iterator);

				uint32_t* tableEntry = &jumpTable[offset];
				duplicates += (*tableEntry != 0) & (*tableEntry == offset);
				*tableEntry = offset;

				//Either we have found a good amount of duplicates, or there are way too many unique e8
				if (duplicates > niceDuplicates || e8count >= maxE8Count)
					break;

				iterator += 4;
				e8count++;
			}

			//Either we found a good amount, or there is a high density of them
			if (duplicates > niceDuplicates || (duplicates > 25 && (double)duplicates / e8count > 0.15)) {

				if (filters.back().filter == X86_FILTER) {
					filters.back().size += thisBlockSize;
				}
				else {
					//Joining together x86 sectors that are close seems to improve ratio a bit, 
					// probably from reducing how often the data changes.
					if (filters.size() > 1 && filters.back().filter == NONE_FILTER &&
						filters.back().size <= (1 << 19) && filters[filters.size() - 2].filter == X86_FILTER) {
						filters[filters.size() - 2].size += filters.back().size + thisBlockSize;
						filters.pop_back();
					}
					else {
						filters.push_back({ X86_FILTER, 0, thisBlockSize });
					}
				}

				input += thisBlockSize;
				continue;
			}

			//  DELTA DETECTION
			uint32_t distanceCount[16];
			std::fill_n(distanceCount, 16, 0);
			const uint8_t* it = thisBlockStart + 16;
#ifdef x64
			for (; it != thisBlockEnd; ) {
				union {
					__m128i partialCountsVec = _mm_setzero_si128();
					uint8_t partialCountsScalar[16];
				};

				const uint8_t* checkpoint = thisBlockEnd - it < 255 ? thisBlockEnd : it + 255;
				for (; it < checkpoint; it++) {
					__m128i cur = _mm_set1_epi8(*it);
					__m128i prev = _mm_loadu_si128((__m128i*)(it - 16));
					__m128i cmp = _mm_cmpeq_epi8(cur, prev);
					//cmpeq returns -1 when they are equal
					partialCountsVec = _mm_sub_epi8(partialCountsVec, cmp);
				}

				for (int i = 0; i < 16; i++)
					distanceCount[15 - i] += partialCountsScalar[i];
			}
#else
			for (; it != thisBlockEnd; it++) {
				distanceCount[1] += it[0] == it[-2];
				distanceCount[2] += it[0] == it[-3];
				distanceCount[3] += it[0] == it[-4];
				distanceCount[4] += it[0] == it[-5];
				distanceCount[5] += it[0] == it[-6];
				distanceCount[6] += it[0] == it[-7];
				distanceCount[7] += it[0] == it[-8];
				distanceCount[8] += it[0] == it[-9];
				distanceCount[9] += it[0] == it[-10];
				distanceCount[10] += it[0] == it[-11];
				distanceCount[11] += it[0] == it[-12];
				distanceCount[12] += it[0] == it[-13];
				distanceCount[13] += it[0] == it[-14];
				distanceCount[14] += it[0] == it[-15];
				distanceCount[15] += it[0] == it[-16];
			}
#endif

			size_t channels = 2;
			size_t highestCount = distanceCount[1];
			for (size_t i = 2; i <= 15; i++) {
				if (distanceCount[i] > highestCount) {
					highestCount = distanceCount[i];
					channels = i + 1;
				}
			}

			//FAST CHECK: compare entropies
			float rawEntropy = calculate_entropy(thisBlockStart, thisBlockSize);
			delta_encode(thisBlockStart, thisBlockSize, deltaBuf, channels);
			float deltaEntropy = calculate_entropy(deltaBuf, thisBlockSize);
			//If we dont have the backend compressor, add a penalty for values equal to 0.
			//These usually help, since that means repeated elements, which compressors usually handle well already.
			if (!backendCompressor)
				deltaEntropy += (float)std::count(deltaBuf, deltaBuf + thisBlockSize, 0) / thisBlockSize * 2.0;

			if (rawEntropy - 0.5 < deltaEntropy) {
				if (filters.back().filter == NONE_FILTER)
					filters.back().size += thisBlockSize;
				else
					filters.push_back({ NONE_FILTER, 0, thisBlockSize });
				input += thisBlockSize;
				continue;
			}

			//PRECISE CHECK: use the backend compressor and compare compressed sizes
			size_t raw, delta;
			if (backendCompressor != nullptr) {
				raw = (*backendCompressor)(thisBlockStart, thisBlockSize);
				delta = (*backendCompressor)(deltaBuf, thisBlockSize);

				if (delta + 128 + std::min(raw / 4, (size_t)1024) > raw) {
					if (filters.back().filter == NONE_FILTER)
						filters.back().size += thisBlockSize;
					else
						filters.push_back({ NONE_FILTER, 0, thisBlockSize });
					input += thisBlockSize;
					continue;
				}
			}

			//If delta encoding helps, try linear prediction
			lpc_encode(thisBlockStart, thisBlockSize, deltaBuf, channels);
			float lpcEntropy = calculate_entropy(deltaBuf, thisBlockSize);
			if (!backendCompressor)
				lpcEntropy += (float)std::count(deltaBuf, deltaBuf + thisBlockSize, 0) / thisBlockSize * 2.0;

			if (deltaEntropy - 0.1 < lpcEntropy) {
				if (filters.back().filter == DELTA_FILTER && filters.back().channels == channels)
					filters.back().size += thisBlockSize;
				else
					filters.push_back({ DELTA_FILTER, channels, thisBlockSize });
				input += thisBlockSize;
				continue;
			}

			if (backendCompressor != nullptr) {
				const size_t lpc = (*backendCompressor)(deltaBuf, thisBlockSize);

				if (lpc + 128 > delta) {
					if (filters.back().filter == DELTA_FILTER && filters.back().channels == channels)
						filters.back().size += thisBlockSize;
					else
						filters.push_back({ DELTA_FILTER, channels, thisBlockSize });
					input += thisBlockSize;
					continue;
				}
			}

			if (filters.back().filter == LPC_FILTER && filters.back().channels == channels)
				filters.back().size += thisBlockSize;
			else
				filters.push_back({ LPC_FILTER, channels, thisBlockSize });
			input += thisBlockSize;
		}

		delete[] deltaBuf;

		input = inputStart;
		const uint8_t* const outputStart = output;

		for (auto filter = filters.begin() + 1; filter != filters.end(); filter++) {

			*output++ = filter->filter;
			write_LEB128(output, filter->size);
			if (filter->filter == DELTA_FILTER || filter->filter == LPC_FILTER)
				*output++ = filter->channels - 1;
		}

		for (auto filter = filters.begin() + 1; filter != filters.end(); filter++) {

			size_t processedSize;
			if (filter->filter == X86_FILTER)
				processedSize = x86_encode(input, filter->size, output);
			else if (filter->filter == DELTA_FILTER)
				processedSize = delta_encode(input, filter->size, output, filter->channels);
			else if (filter->filter == LPC_FILTER)
				processedSize = lpc_encode(input, filter->size, output, filter->channels);
			//raw data
			else {
				memcpy(output, input, filter->size);
				processedSize = filter->size;
			}
			if (processedSize == -1)
				return -1;
			output += processedSize;
			input += filter->size;
		}

		return output - outputStart;
	}

	size_t spectrum_bound(size_t size) {
		return size + size / 7 + 1024;
	}

	int spectrum_decode(const uint8_t* encoded, const size_t encodedSize, uint8_t* decoded, const size_t decodedSize) {

		const uint8_t* const decodedStart = decoded;
		const uint8_t* const decodedEnd = decoded + decodedSize;
		const uint8_t* const encodedEnd = encoded + encodedSize;

		std::vector<PreprocessorBlock> filters;
		try {
			size_t detectedSize = 0;  //total bytes stored by currently read filters
			while (detectedSize < decodedSize) {
				size_t blockSize, channels, filter;
				if (read_LEB128(encoded, encodedEnd, &filter))
					return -1;
				if (read_LEB128(encoded, encodedEnd, &blockSize))
					return -1;
				if (filter == DELTA_FILTER || filter == LPC_FILTER) {
					if (encoded == encodedEnd)
						return -1;
					channels = (*encoded++) + 1;
				}
				filters.push_back({ filter, channels, blockSize });

				if (decodedSize - detectedSize < blockSize)
					return -1;
				detectedSize += blockSize;
			}
		}
		catch (std::bad_alloc& e) {
			return -1;
		}

		for (auto filter = filters.begin(); filter != filters.end(); filter++) {

			size_t processedSize;
			if (filter->filter == X86_FILTER) 
				processedSize = x86_decode(encoded, encodedEnd, filter->size, decoded);
			else if (filter->filter == DELTA_FILTER)
				processedSize = delta_decode(encoded, filter->size, decoded, filter->channels);
			else if (filter->filter == LPC_FILTER)
				processedSize = lpc_decode(encoded, filter->size, decoded, filter->channels);
			//raw data
			else {
				memcpy(decoded, encoded, filter->size);
				processedSize = filter->size;
			}
			if (processedSize == -1)
				return -1;
			encoded += processedSize;
			decoded += filter->size;
		}

		return 0;
	}
}

#endif //SPECTRUM_IMPLEMENTATION

#endif //__SPECTRUM__
