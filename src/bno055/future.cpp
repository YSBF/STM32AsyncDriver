

#include "future"
#include <condition_variable>
#include <system_error>
#include <atomic>
#include <bits/atomic_futex.h>
#include <bits/functexcept.h>
#include <bits/invoke.h>
#include <bits/unique_ptr.h>
#include <bits/shared_ptr.h>
#include <bits/std_function.h>
#include <bits/uses_allocator.h>
#include <bits/allocated_ptr.h>
#include <ext/aligned_buffer.h>
#include <tuple>


template<typename _Fn, typename... _Args>
using __async_result_of = typename std::__invoke_result<typename std::decay<_Fn>::type, typename std::decay<_Args>::type...>::type;

template<typename _Fn, typename... _Args>
inline std::future<__async_result_of<_Fn, _Args...>>
async(_Fn &&__fn, _Args &&... __args) {
  return std::async(std::launch::async | std::launch::deferred,
                    std::forward<_Fn>(__fn),
                    std::forward<_Args>(__args)...);
}

