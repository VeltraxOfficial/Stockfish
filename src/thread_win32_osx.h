/*
  Stockfish, a UCI chess playing engine derived from Glaurung 2.1
  Copyright (C) 2004-2025 The Stockfish developers (see AUTHORS file)

  Stockfish is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Stockfish is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef THREAD_WIN32_OSX_H_INCLUDED
#define THREAD_WIN32_OSX_H_INCLUDED

#include <thread>
#include <new>
#include <system_error>
#include <cstdlib>
#include <vector>

// On OSX threads other than the main thread are created with a reduced stack
// size of 512KB by default, this is too low for deep searches, which require
// somewhat more than 1MB stack, so adjust it to TH_STACK_SIZE.
// The implementation calls pthread_create() with the stack size parameter
// equal to the Linux 8MB default, on platforms that support it.

#if defined(__APPLE__) || defined(__MINGW32__) || defined(__MINGW64__) || defined(USE_PTHREADS)

    #include <pthread.h>
    #include <functional>

namespace Stockfish {

class NativeThread {
    pthread_t thread;

    static constexpr size_t TH_STACK_SIZE = 8 * 1024 * 1024;

   public:
    template<class Function, class... Args>
    explicit NativeThread(Function&& fun, Args&&... args) {
        auto func_up = std::make_unique<std::function<void()>>(
          std::bind(std::forward<Function>(fun), std::forward<Args>(args)...));
        std::function<void()>* func = func_up.release();

        pthread_attr_t attr_storage, *attr = &attr_storage;
        pthread_attr_init(attr);
        pthread_attr_setstacksize(attr, TH_STACK_SIZE);

        auto start_routine = [](void* ptr) -> void* {
            std::unique_ptr<std::function<void()>> f(reinterpret_cast<std::function<void()>*>(ptr));
            (*f)();
            return nullptr;
        };

        int rc = pthread_create(&thread, attr, start_routine, func);
        pthread_attr_destroy(attr);
        if (rc != 0) {
            delete func;
            throw std::system_error(rc, std::generic_category(), "pthread_create failed");
        }
    }

    void join() { pthread_join(thread, nullptr); }
};

}  // namespace Stockfish

#else  // Default case: use STL classes

namespace Stockfish {

using NativeThread = std::thread;

}  // namespace Stockfish

#endif

#ifdef _WIN32
#include <windows.h>
#include <vector>
#include <unordered_map>
#include <memory>
#include <mutex>

static const std::vector<GROUP_AFFINITY>& sf_perf_core_affinities() {
    static std::vector<GROUP_AFFINITY> perf;
    static std::once_flag initFlag;
    std::call_once(initFlag, []() {
        DWORD len = 0;
        BOOL ok = GetLogicalProcessorInformationEx(RelationProcessorCore, nullptr, &len);
        if (ok || GetLastError() != ERROR_INSUFFICIENT_BUFFER)
            return;
        std::vector<char> buffer(len);
        PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX info =
            reinterpret_cast<PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX>(buffer.data());
        if (!GetLogicalProcessorInformationEx(RelationProcessorCore, info, &len))
            return;
        BYTE maxClass = 0;
        BYTE* base = reinterpret_cast<BYTE*>(info);
        DWORD offset = 0;
        while (offset < len) {
            PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX item =
                reinterpret_cast<PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX>(base + offset);
            if (item->Relationship == RelationProcessorCore) {
                if (item->Processor.EfficiencyClass > maxClass)
                    maxClass = item->Processor.EfficiencyClass;
            }
            offset += item->Size;
        }
        if (maxClass == 0)
            return;
        std::unordered_map<WORD, KAFFINITY> maskMap;
        offset = 0;
        while (offset < len) {
            PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX item =
                reinterpret_cast<PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX>(base + offset);
            if (item->Relationship == RelationProcessorCore
                && item->Processor.EfficiencyClass == maxClass) {
                WORD gc = item->Processor.GroupCount;
                for (WORD i = 0; i < gc; ++i) {
                    const GROUP_AFFINITY& gm = item->Processor.GroupMask[i];
                    maskMap[gm.Group] |= gm.Mask;
                }
            }
            offset += item->Size;
        }
        for (auto &kv : maskMap) {
            GROUP_AFFINITY ga = {};
            ga.Group = kv.first;
            ga.Mask = kv.second;
            perf.push_back(ga);
        }
    });
    return perf;
}

static inline void sf_apply_pcore_affinity_to_current_thread() {
    const char *env = std::getenv("SF_PIN_PCORES");
    if (!env || env[0] != '1')
        return;

    const auto &perf = sf_perf_core_affinities();
    if (perf.empty())
        return;
    GROUP_AFFINITY curr = {};
    if (!GetThreadGroupAffinity(GetCurrentThread(), &curr))
        return;
    for (const auto &ga : perf) {
        if (ga.Group == curr.Group) {
            if (SetThreadGroupAffinity(GetCurrentThread(), &ga, nullptr))
                return;
            else
                return;
        }
    }
    for (const auto &ga : perf) {
        if (SetThreadGroupAffinity(GetCurrentThread(), &ga, nullptr))
            return;
    }
}
#endif

#endif  // #ifndef THREAD_WIN32_OSX_H_INCLUDED
