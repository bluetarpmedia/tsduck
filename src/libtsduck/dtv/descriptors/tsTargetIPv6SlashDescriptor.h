//----------------------------------------------------------------------------
//
// TSDuck - The MPEG Transport Stream Toolkit
// Copyright (c) 2005-2023, Thierry Lelegard
// BSD-2-Clause license, see LICENSE.txt file or https://tsduck.io/license
//
//----------------------------------------------------------------------------
//!
//!  @file
//!  Representation of a target_IPv6_slash_descriptor (INT specific).
//!
//----------------------------------------------------------------------------

#pragma once
#include "tsAbstractDescriptor.h"
#include "tsIPv6Address.h"
#include "tsIPUtils.h"

namespace ts {
    //!
    //! Representation of a target_IPv6_slash_descriptor (INT specific).
    //!
    //! This descriptor cannot be present in other tables than an INT
    //! because its tag reuses an MPEG-defined one.
    //!
    //! @see ETSI EN 301 192, 8.4.5.12
    //! @ingroup descriptor
    //!
    class TSDUCKDLL TargetIPv6SlashDescriptor : public AbstractDescriptor
    {
    public:
        //!
        //! Structure of an address entry in the descriptor.
        //!
        class Address
        {
        public:
            IPv6Address IPv6_addr;         //!< IPv6 address.
            uint8_t     IPv6_slash_mask;   //!< Number of bits in network mask.

            //!
            //! Constructor
            //! @param [in] addr IPv6 address.
            //! @param [in] mask Number of bits in network mask.
            //!
            Address(const IPv6Address& addr = IPv6Address(), uint8_t mask = 0);
        };

        // TargetIPv6SlashDescriptor public members:
        std::vector<Address> addresses;  //!< IPv6 addresses

        //!
        //! Maximum number of entries to fit in 255 bytes.
        //!
        static const size_t MAX_ENTRIES = 15;

        //!
        //! Default constructor.
        //!
        TargetIPv6SlashDescriptor();

        //!
        //! Constructor from a binary descriptor.
        //! @param [in,out] duck TSDuck execution context.
        //! @param [in] bin A binary descriptor to deserialize.
        //!
        TargetIPv6SlashDescriptor(DuckContext& duck, const Descriptor& bin);

        // Inherited methods
        DeclareDisplayDescriptor();

    protected:
        // Inherited methods
        virtual void clearContent() override;
        virtual void serializePayload(PSIBuffer&) const override;
        virtual void deserializePayload(PSIBuffer&) override;
        virtual void buildXML(DuckContext&, xml::Element*) const override;
        virtual bool analyzeXML(DuckContext&, const xml::Element*) override;
    };
}
