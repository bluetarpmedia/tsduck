//----------------------------------------------------------------------------
//
// TSDuck - The MPEG Transport Stream Toolkit
// Copyright (c) 2022-2023, Paul Higgs
// BSD-2-Clause license, see LICENSE.txt file or https://tsduck.io/license
//
//----------------------------------------------------------------------------
//!
//!  @file
//!  Representation of an MPEG-4 Text Descriptor
//!
//----------------------------------------------------------------------------

#pragma once
#include "tsAbstractDescriptor.h"
#include "tsByteBlock.h"
#include "tsVariable.h"

namespace ts {
    //!
    //! Representation of an MPEG-4 Text Descriptor
    //! @see ITU-T H.222.0 clause 2.6.70 and ISO/IEC 14496-17
    //! @ingroup descriptor
    //!
    class TSDUCKDLL MPEG4TextDescriptor : public AbstractDescriptor
    {
    public:
        //!
        //! TextConfig entry.
        //! @see ISO/IEC 14496-17, clause 5.2
        //!
        class TSDUCKDLL TextConfig_type
        {
        public:
            uint8_t              textFormat;               //!< 8 bits, ISO/IEC 14496-17, clause 5.2
            ByteBlock            formatSpecificTextConfig; //!< 16 bits, ISO/IEC 14496-17, clause 5.2

            TextConfig_type();                      //!< Constructor
        };

        //!
        //! Sample_index_and_description entry.
        //! @see ISO/IEC 14496-17, clause 7.5
        //!
        class TSDUCKDLL Sample_index_and_description_type
        {
        public:
            uint8_t              sample_index;            //!< 8 bits, ISO/IEC 14496-17, clause 7.5
            TextConfig_type      SampleDescription;       //!< ISO/IEC 14496-17, clause 7.5


            Sample_index_and_description_type();    //!< Constructor
        };

        // Public members:
        uint8_t            textFormat;              //!< 8 bits, ISO/IEC 14496-17, clause 5.2
        uint8_t            ThreeGPPBaseFormat;      //!< 8 bits, ISO/IEC 14496-17, clause 7.5
        uint8_t            profileLevel;            //!< 8 bits, ISO/IEC 14496-17, clause 7.5
        uint32_t           durationClock;           //!< 24 bits, ISO/IEC 14496-17, clause 7.5
        uint8_t            sampleDescriptionFlags;  //!< 2 bits, ISO/IEC 14496-17, clause 7.5
        uint8_t            layer;                   //!< 2 bits, ISO/IEC 14496-17, clause 7.5
        uint16_t           text_track_width;        //!< 16 bits, ISO/IEC 14496-17, clause 7.5
        uint16_t           text_track_height;       //!< 16 bits, ISO/IEC 14496-17, clause 7.5
        ByteBlock          Compatible_3GPPFormat;   //!< list of 8 bit values, ISO/IEC 14496-17, clause 7.5
        Variable<uint16_t> scene_width;             //!< 16 bits, ISO/IEC 14496-17, clause 7.5
        Variable<uint16_t> scene_height;            //!< 16 bits, ISO/IEC 14496-17, clause 7.5
        Variable<uint16_t> horizontal_scene_offset; //!< 16 bits, ISO/IEC 14496-17, clause 7.5
        Variable<uint16_t> vertical_scene_offset;   //!< 16 bits, ISO/IEC 14496-17, clause 7.5

        std::vector<Sample_index_and_description_type> Sample_index_and_description;   //!< list of sample indexes, ISO/IEC 14496-17, clause 7.5

        //!
        //! Default constructor.
        //!
        MPEG4TextDescriptor();

        //!
        //! Constructor from a binary descriptor
        //! @param [in,out] duck TSDuck execution context.
        //! @param [in] bin A binary descriptor to deserialize.
        //!
        MPEG4TextDescriptor(DuckContext& duck, const Descriptor& bin);

        // Inherited methods
        DeclareDisplayDescriptor();

    private:
        static UString TimedText_TS26245(ByteBlock formatSpecificTextConfig);

    protected:
        // Inherited methods
        virtual void clearContent() override;
        virtual void serializePayload(PSIBuffer&) const override;
        virtual void deserializePayload(PSIBuffer&) override;
        virtual void buildXML(DuckContext&, xml::Element*) const override;
        virtual bool analyzeXML(DuckContext&, const xml::Element*) override;

    private:
        static const std::vector<uint8_t> allowed_textFormat_values;
        static const std::vector<uint8_t> allowed_3GPPBaseFormat_values;
        static const std::vector<uint8_t> allowed_profileLevel_values;
    };
}
