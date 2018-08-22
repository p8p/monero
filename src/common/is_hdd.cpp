// Copyright (c) 2014-2018, The Monero Project
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are
// permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of
//    conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice, this list
//    of conditions and the following disclaimer in the documentation and/or other
//    materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors may be
//    used to endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
// THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
// THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Parts of this file are originally copyright (c) 2012-2013 The Cryptonote developers
#include "is_hdd.h"
#include <cstdlib>
#if defined(__GLIBC__)
  #include <sys/stat.h>
  #include <sstream>
  #include <sys/sysmacros.h>
  #include <fstream>
#elif defined(_WIN32) and (_WIN32_WINNT >= 0x0601)
  #include <windows.h>
  #include <winioctl.h>
  #include <boost/range/iterator_range.hpp>
  #include <boost/spirit/include/qi_raw.hpp>
  #include <boost/spirit/include/qi_char_.hpp>
  #include <boost/spirit/include/qi_lit.hpp>
  #include <boost/spirit/include/qi_difference.hpp>
  #include <boost/spirit/include/qi_kleene.hpp>
  #include <boost/spirit/include/qi_sequence.hpp>
#endif
#include <boost/optional.hpp>
namespace tools
{
#if defined(__GLIBC__)
  boost::optional<bool> is_hdd_sysfs(const char *file_path)
  {
    struct stat st;
    std::string prefix;
    if(stat(file_path, &st) == 0)
    {
      std::ostringstream s;
      s << "/sys/dev/block/" << major(st.st_dev) << ":" << minor(st.st_dev);
      prefix = s.str();
    }
    else
    {
      return boost::none;
    }
    std::string attr_path = prefix + "/queue/rotational";
    std::ifstream f(attr_path, std::ios_base::in);
    if(not f.is_open())
    {
      attr_path = prefix + "/../queue/rotational";
      f.open(attr_path, std::ios_base::in);
      if(not f.is_open())
      {
          return boost::none;
      }
    }
    unsigned short val = 0xdead;
    f >> val;
    if(not f.fail())
    {
      return (val == 1);
    }
    return boost::none;
  }
#elif defined(_WIN32) and (_WIN32_WINNT >= 0x0601)

  struct close_handle
  {
    void operator()(HANDLE handle) const noexcept
    {
      CloseHandle(handle);
    }
  };
  //file path to logical volume
  boost::optional<std::string> fp2lv(const char *fp)
  {
    std::unique_ptr<void, close_handle> h_ptr;
    HANDLE h = CreateFile(
      fp,
      0,
      FILE_SHARE_READ | FILE_SHARE_WRITE,
      nullptr,
      OPEN_EXISTING,
      FILE_ATTRIBUTE_NORMAL | FILE_FLAG_BACKUP_SEMANTICS,
      nullptr
      );
    h_ptr.reset(h);
    if(h != INVALID_HANDLE_VALUE)
    {
      DWORD p_size = GetFinalPathNameByHandleA(
        h,
        nullptr,
        0,
        VOLUME_NAME_NT | FILE_NAME_NORMALIZED
        );
      std::string p(p_size, char{});
      DWORD r_size = GetFinalPathNameByHandleA(
        h,
        &p[0],
        p_size + 1,
        VOLUME_NAME_NT | FILE_NAME_NORMALIZED
        );
      namespace qi = boost::spirit::qi;
      boost::iterator_range<std::string::const_iterator> m{};
      bool success = qi::parse(
        p.cbegin(),
        p.cend(),
        (qi::lit("\\Device\\") >> qi::raw[*(qi::char_ - "\\")]),
        m
        );
      if(success and not m.empty())
      {
        return std::string(m.begin(), m.end());
      }
    }
    return boost::none;
  }

  //logical volume to physical volumes
  bool lv2pv(const char *lv, std::vector<std::string> &pvs)
  {
    std::string lv_path = "\\\\?\\";
    lv_path += lv;
    std::unique_ptr<void, close_handle> h_ptr;
    HANDLE h = CreateFile(
      lv_path.c_str(),
      0,
      FILE_SHARE_READ | FILE_SHARE_WRITE,
      nullptr,
      OPEN_EXISTING,
      FILE_ATTRIBUTE_NORMAL,
      nullptr
      );
    h_ptr.reset(h);
    if(h != INVALID_HANDLE_VALUE)
    {
      VOLUME_DISK_EXTENTS r;
      DWORD r_size = 0;
      BOOL success = DeviceIoControl(
        h,
        IOCTL_VOLUME_GET_VOLUME_DISK_EXTENTS,
        nullptr,
        0,
        &r,
        sizeof(r),
        &r_size,
        nullptr
        );
      if(success and r_size == sizeof(r))
      {
        std::ostringstream ss;
        if(r.NumberOfDiskExtents > 0)
        {
          ss << "PhysicalDrive" << r.Extents[0].DiskNumber;
          pvs.push_back(ss.str());
          for(uint32_t i = 1; i < r.NumberOfDiskExtents; ++i)
          {
            ss.str(std::string{});
            ss << "PhysicalDrive" << r.Extents[i].DiskNumber;
            pvs.push_back(ss.str());
          }
        }
        return true;
      }
    }
    return false;
  }

  boost::optional<bool> win_device_has_seek_penalty(const char *pv)
  {
    std::string pv_path = "\\\\?\\";
    pv_path += pv;
    std::unique_ptr<void, close_handle> h_ptr;
    HANDLE h = CreateFile(
      pv_path.c_str(),
      0,
      FILE_SHARE_READ | FILE_SHARE_WRITE,
      nullptr,
      OPEN_EXISTING,
      FILE_ATTRIBUTE_NORMAL,
      nullptr
      );
    h_ptr.reset(h);
    if(h != INVALID_HANDLE_VALUE)
    {
      STORAGE_PROPERTY_QUERY q{
        .PropertyId = StorageDeviceSeekPenaltyProperty,
        .QueryType = PropertyStandardQuery
      };
      DEVICE_SEEK_PENALTY_DESCRIPTOR r;
      DWORD r_size = 0;
      BOOL success = DeviceIoControl(
        h,
        IOCTL_STORAGE_QUERY_PROPERTY,
        &q,
        sizeof(q),
        &r,
        sizeof(r),
        &r_size,
        nullptr
        );
      if(success and r_size == sizeof(r))
      {
        return r.IncursSeekPenalty;
      }
    }
    return boost::none;
  }

  boost::optional<bool> is_hdd_win_ioctl(const char *path)
  {
    boost::optional<std::string> lv = fp2lv(path);
    if(not lv)
    {
      return boost::none;
    }
    std::vector<std::string> pvs;
    bool pv_success = lv2pv(lv.value().c_str(), pvs);
    if(not pv_success)
    {
      return boost::none;
    }
    for(const auto &v: pvs)
    {
      boost::optional<bool> q = win_device_has_seek_penalty(v.c_str());
      if(not q)
      {
        return boost::none;
      }
      if(q.value())
      {
        return true;
      }
    }
    return false;
  }
#endif
  boost::optional<bool> is_hdd(const char *path)
  {
    #if defined(_WIN32) and (_WIN32_WINNT >= 0x0601)
    return is_hdd_win_ioctl(path);
    #elif defined(__GLIBC__)
    return is_hdd_sysfs(path);
    #else
    return boost::none;
    #endif
  }
}
