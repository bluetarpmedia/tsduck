//----------------------------------------------------------------------------
//
// TSDuck - The MPEG Transport Stream Toolkit
// Copyright (c) 2022-2023, Vision Advance Technology Inc. (VATek)
// BSD-2-Clause license, see LICENSE.txt file or https://tsduck.io/license
//
//----------------------------------------------------------------------------
//!
//!  @file
//!  Declare the ts::VatekOutputPlugin class.
//!
//----------------------------------------------------------------------------

#pragma once
#if !defined(TS_NO_VATEK) || defined(DOXYGEN)

#include "tsOutputPlugin.h"

namespace ts {
    //!
    //! Vatek output plugin for @c tsp.
    //! @ingroup plugin
    //!
    class TSDUCKDLL VatekOutputPlugin: public OutputPlugin
    {
        TS_NOBUILD_NOCOPY(VatekOutputPlugin);
    public:
        //!
        //! Constructor.
        //! @param [in] tsp Associated callback to @c tsp executable.
        //!
        VatekOutputPlugin(TSP* tsp);

        //!
        //! Destructor.
        //!
        virtual ~VatekOutputPlugin() override;

        // Implementation of plugin API
        virtual bool start() override;
        virtual bool stop() override;
        virtual bool send(const TSPacket*, const TSPacketMetadata*, size_t) override;
        virtual bool isRealTime() override;
        virtual BitRate getBitrate() override;
        virtual BitRateConfidence getBitrateConfidence() override;

    private:
        class Guts;
        Guts* _guts;
    };
}

#endif // TS_NO_VATEK
