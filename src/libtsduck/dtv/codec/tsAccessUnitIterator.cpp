//----------------------------------------------------------------------------
//
// TSDuck - The MPEG Transport Stream Toolkit
// Copyright (c) 2005-2023, Thierry Lelegard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
//----------------------------------------------------------------------------

#include "tsAccessUnitIterator.h"
#include "tsPESPacket.h"
#include "tsAVC.h"
#include "tsHEVC.h"
#include "tsVVC.h"

#ifdef _WIN64
    #include <emmintrin.h>
#endif

//----------------------------------------------------------------------------
// Constructors.
//----------------------------------------------------------------------------

ts::AccessUnitIterator::AccessUnitIterator(const uint8_t* data, size_t size, uint8_t stream_type, CodecType default_format) :
    _data(data),
    _data_size(size),
    _valid(PESPacket::HasCommonVideoHeader(data, size)),
    _format(_valid ? default_format : CodecType::UNDEFINED),
    _nalunit(nullptr),
    _nalunit_size(0),
    _nalunit_header_size(0),
    _nalunit_index(0),
    _nalunit_type(AVC_AUT_INVALID)
{
    // If the area is valid, compute the actual format.
    if (_valid) {
        // Determine encoding from stream type in PMT.
        // If unspecified, keep default format.
        if (StreamTypeIsAVC(stream_type)) {
            _format = CodecType::AVC;
        }
        else if (StreamTypeIsHEVC(stream_type)) {
            _format = CodecType::HEVC;
        }
        else if (StreamTypeIsVVC(stream_type)) {
            _format = CodecType::VVC;
        }
        else if (stream_type != ST_NULL || (_format != CodecType::AVC && _format != CodecType::HEVC && _format != CodecType::VVC)) {
            // This is an explicit but unsupported stream or codec type.
            _format = CodecType::UNDEFINED;
            _valid = false;
        }
    }

    // Search the first access unit.
    reset();
}


//----------------------------------------------------------------------------
// Reset the exploration of the data area at the beginning.
//----------------------------------------------------------------------------

void ts::AccessUnitIterator::reset()
{
    if (_valid) {
        // Point to the beginning of area, before the first access unit.
        // Calling next() will find the first one (if any).
        _nalunit = _data;
        next();
        // Reset NALunit index since we point to the first one.
        _nalunit_index = 0;
    }
}


//----------------------------------------------------------------------------
// Check if the current access unit is a Supplemental Enhancement Information
//----------------------------------------------------------------------------

bool ts::AccessUnitIterator::currentAccessUnitIsSEI() const
{
    // Not all enum values used in switch, intentionally.
    TS_PUSH_WARNING()
    TS_LLVM_NOWARNING(switch-enum)
    TS_MSC_NOWARNING(4061)

    switch (_format) {
        case CodecType::AVC: return _nalunit_type == AVC_AUT_SEI;
        case CodecType::HEVC: return _nalunit_type == HEVC_AUT_PREFIX_SEI_NUT || _nalunit_type == HEVC_AUT_SUFFIX_SEI_NUT;
        case CodecType::VVC: return _nalunit_type == VVC_AUT_PREFIX_SEI_NUT || _nalunit_type == VVC_AUT_SUFFIX_SEI_NUT;
        default: return false;
    }

    TS_POP_WARNING()
}


//----------------------------------------------------------------------------
// Iterate to the next access unit.
//----------------------------------------------------------------------------

namespace {

enum class FindMode
{
    Find_001,
    Find_001_or_000
};

const uint8_t* FindStartCode(const uint8_t *stream, int32_t streamLength, FindMode findMode = FindMode::Find_001, int32_t searchOffset = 0)
{
    if (searchOffset < 0)
    {
        return nullptr;
    }

    const auto Find = [findMode](const uint8_t *data, int32_t dataLength, int32_t maxSearchLength = 0) -> const uint8_t *
    {
        int32_t bytesSearched = 0;

        if (findMode == FindMode::Find_001_or_000)
        {
            while (dataLength >= 3)
            {
                if (data[0] == 0x0 && data[1] == 0x0 && data[2] <= 0x1)  // Note the <= to find 0 or 1 in third byte
                    return data;

                ++data;
                --dataLength;

                ++bytesSearched;
                if (maxSearchLength > 0 && bytesSearched > maxSearchLength)
                    return nullptr;
            }
        }
        else
        {
            while (dataLength >= 3)
            {
                if (data[0] == 0x0 && data[1] == 0x0 && data[2] == 0x1)
                    return data;

                ++data;
                --dataLength;

                ++bytesSearched;
                if (maxSearchLength > 0 && bytesSearched > maxSearchLength)
                    return nullptr;
            }
        }

        return nullptr; // not found
    };

    const int32_t searchLength = streamLength - searchOffset;

#ifdef _WIN64

    // If the search range is small or the `stream` pointer is not 16-byte-aligned then use the simple method.
    if (searchLength < 32 || ptrdiff_t(stream) % 16 != 0)
    {
        return Find(&stream[searchOffset], searchLength);
    }

    int32_t i = searchOffset;

    // Pre-roll until stream[i] is 16-byte-aligned
    const auto preroll = ptrdiff_t(stream + searchOffset) % 16;
    if (preroll != 0)
    {
        const int32_t end = searchOffset + 16 - int32_t(preroll);
        assert(end % 16 == 0);
        bool foundZero = false;
        for (; i < end; ++i)
        {
            if (stream[i] == 0)
            {
                foundZero = true;
                break;
            }
        }

        if (foundZero)
        {
            if (const auto ptr = Find(&stream[searchOffset], searchLength, 24))  // One line plus spill into next line in case startcode straddles
            {
                return ptr;
            }

            // If we broke out of the loop and Find didn't return a startcode position 
            // then set `i` to `end`, because we've already searched all these bytes.
            i = end;
        }
    }

    assert(i % 16 == 0);

    // SSE2
    __m128i vecLine{}, vecMask1{}, vecMask2{};
    static const __m128i vecZero = _mm_setzero_si128();
    static const __m128i vecZeroOne = _mm_set_epi8(1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0);

    // Read 16 bytes at a time into `vecLine`. Then compare with a "0,0" and "0,1" vectors to try
    // and find a startcode.
    for (; i < streamLength - 15; i += 16)
    {
        vecLine = _mm_load_si128((__m128i *) & stream[i]);
        vecMask1 = _mm_cmpeq_epi8(vecLine, vecZero);         // Sets 0xFF for each 8-bit element if equal
        vecMask2 = _mm_cmpeq_epi8(vecLine, vecZeroOne);

        const uint32_t mask1 = _mm_movemask_epi8(vecMask1);  // Take the most significant bit of each 8-bit element and create a new 32-bit mask
        const uint32_t mask2 = _mm_movemask_epi8(vecMask2);

        // Test for two adjacent bits set in each mask
        const bool foundZeroZero = ((mask1 & (mask1 >> 1)) > 0);
        const bool foundZeroOne = ((mask2 & (mask2 >> 1)) > 0);
        const bool lineStartsWithZeroOne = foundZeroOne && (mask2 & 0x3);

        // First, check if the current line begins with "0,1" (in case previous line ends with "0"), since
        // this would be the first possible startcode we could find (at position i-1). If so, we search the
        // entire 17 bytes (last pos + this line) and hence don't need to also test the `foundZeroZero` case.
        // There's no need to test for "0,1" anywhere else in this line because we either find "0,1" at the start,
        // and hence search the whole line, or we proceed to the `foundZeroZero` case which will detect a possible
        // match.
        if (lineStartsWithZeroOne)
        {
            const int32_t previousPos = (std::max)(i - 1, searchOffset);  // Clamp to `searchOffset` since the client doesn't want us looking prior to this
            assert(previousPos >= 0 && previousPos >= searchOffset);

            if (const auto ptr = Find(&stream[previousPos], streamLength - i, 17))  // Search from prev pos (1 byte) + this line (16 bytes)
                return ptr;
        }
        else if (foundZeroZero)
        {
            if (const auto ptr = Find(&stream[i], streamLength - i, 17))  // Search this line (16 bytes) plus the start of next line (in case this line ends with "0,0").
                return ptr;
        }
    }

    // Handle remaining bytes if the range wasn't cleanly divisible by zero
    if (i < streamLength)
    {
        if (const auto ptr = Find(&stream[i], streamLength - i))
            return ptr;
    }

    return nullptr;

#else

    return Find(&stream[searchOffset], searchLength);

#endif
}

} // anon

bool ts::AccessUnitIterator::next()
{
    // Cannot iterate on invalid area ar after end of iteration..
    if (!_valid || _nalunit == nullptr) {
        return false;
    }

    // Memory patterns which are used between access units.
    static const uint8_t Zero3[] = {0x00, 0x00, 0x00};
    static const uint8_t StartCodePrefix[] = {0x00, 0x00, 0x01};

    // Remaining size in data area.
    assert(_nalunit >= _data);
    assert(_nalunit <= _data + _data_size);
    size_t remain = _data + _data_size - _nalunit;

    // Preset access unit type to an invalid value.
    // If the video format is undefined, we won't be able to extract a valid one.
    _nalunit_type = AVC_AUT_INVALID;
    _nalunit_size = 0;
    _nalunit_header_size = 0;

    // Locate next access unit: starts with 00 00 01.
    // The start code prefix 00 00 01 is not part of the NALunit.
    // The NALunit starts at the NALunit type byte (see H.264, 7.3.1).
    const uint8_t* const p1 = FindStartCode(_nalunit, int32_t(remain));
    if (p1 == nullptr) {
        // No next access unit.
        _nalunit = nullptr;
        _nalunit_index++;
        return false;
    }

    // Jump to first byte of NALunit.
    remain -= p1 - _nalunit + sizeof(StartCodePrefix);
    _nalunit = p1 + sizeof(StartCodePrefix);

    // Locate end of access unit: ends with 00 00 00, 00 00 01 or end of data.
    const uint8_t *const p2 = FindStartCode(_nalunit, int32_t(remain), FindMode::Find_001_or_000);
    if (p2)
    {
        // NALunit ends at 00 00 01.
        assert(p2 != nullptr);
        _nalunit_size = p2 - _nalunit;
    }
    else
    {
        // No 00 00 01, no 00 00 00, the NALunit extends up to the end of data.
        _nalunit_size = remain;
    }

    // Extract NALunit type.
    if (_format == CodecType::AVC && _nalunit_size >= 1) {
        _nalunit_header_size = 1;
        _nalunit_type = _nalunit[0] & 0x1F;
    }
    else if (_format == CodecType::HEVC && _nalunit_size >= 1) {
        _nalunit_header_size = 2;
        _nalunit_type = (_nalunit[0] >> 1) & 0x3F;
    }
    else if (_format == CodecType::VVC && _nalunit_size >= 2) {
        _nalunit_header_size = 2;
        _nalunit_type = (_nalunit[1] >> 3) & 0x1F;
    }

    // Count NALunits.
    _nalunit_index++;
    return true;
}
