#pragma once

#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_util.hpp"    // for vfc::nop

namespace Y163
{

namespace daddy
{
struct CInterfaceBase
{
     public:
    enum { BASE_VERSION = 0 };

    /// @brief c'tor.
    CInterfaceBase()
        : m_referenceCounter(0)
        , m_sequenceNumber(0)
    {
    }

    /// @brief Copy Constructor
    ///
    /// The below copy constructor is needed to attain the instance of an existance object
    /// to a newly created object. Here the member values reinitiliased to 0, since the
    /// member variables are internal DADDY management data
    /// @note: This is needed atleast for Copy Runnable concept.
    CInterfaceBase(const CInterfaceBase& that)
        : m_referenceCounter(0)
        , m_sequenceNumber(0)
    {
        vfc::nop(that);
    }

    /// @brief Operator Overloading
    ///
    /// The operator overloading is needed to avoid the overriding of member values.
    /// @note: This is needed atleast for Copy Runnable concept
    CInterfaceBase& operator=(const CInterfaceBase& that)
    {
        vfc::nop(that);
        return *this;
    }

    bool operator==(const CInterfaceBase& that) const
    {
        return m_referenceCounter == that.m_referenceCounter && m_sequenceNumber == that.m_sequenceNumber;
    }

    ~CInterfaceBase()
    {
    }

    vfc::uint16_t m_referenceCounter;
    vfc::uint16_t m_sequenceNumber;
};
}
}
