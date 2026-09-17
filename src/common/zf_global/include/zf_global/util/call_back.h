
/******************************************************************************
 * Copyright 2023 ZF. All Rights Reserved.
 * Author:
 *        xuheng.chai@zf.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
/**
 * @file
 * @brief Defines the cycle queue.
 * @return
 */

#ifndef ZF_GLOBAL_DELEGATE_H
#define ZF_GLOBAL_DELEGATE_H

#include <functional>
#include <memory>
#include <iterator>
#include "zf_global/zf_global.h"

BEGIN_NS_ZF

template <typename T, std::size_t N>
void PrintArray(const T(&arr)[N], std::ostream& os) {
    std::copy(std::begin(arr), std::end(arr), std::ostream_iterator<T>(os, " "));
    std::cout << std::endl;
}

template <typename T, std::size_t N>
void PrintArray(const T(&arr)[N]) {
    std::copy(std::begin(arr), std::end(arr), std::ostream_iterator<T>(std::cout, " "));
    std::cout << std::endl;
}

class DelegateBase
{
};

/*
 * @param R: Return type
 * @param ... :template for variable args
 */
template <class R, typename... Args> // typename R = void
class Delegate : public DelegateBase
{
public:
    using CallbackType = std::function<R(Args...)>;
    Delegate(CallbackType func) : m_func(func) {}
    Delegate() {}
    R operator()(Args ...args)
    {
        return m_func(std::forward<Args>(args)...);
    }
    R InvokeCallback(Args ...args)
    {
        return m_func(std::forward<Args>(args)...);
    }
    void RegisterCallback(CallbackType fun)
    {
        m_func = fun;
    }

private:
    CallbackType m_func;
};

template <class R, class T, typename... Args>
Delegate<R, Args...> CreateDelegate(T *t, R (T::*f)(Args...))
{
    // std::function<R(Args...)> func = std::bind(f, t);// not work
    auto func = [t, f](Args... args)
    { return (t->*f)(std::forward<Args>(args)...); };
    return Delegate<R, Args...>(func);
}

template <class R, typename... Args>
Delegate<R, Args...> CreateDelegate(R (*f)(Args...))
{
    return Delegate<R, Args...>(f);
}

class CallBackBase
{
public:
    template <class R, typename... Args>
    void RegisterCallback(std::function<R(Args...)> fun)
    {
        //std::cout << "begin RegisterCallback" << std::endl;
        auto cb = Delegate<R, Args...>(fun);
        m_callBack = std::make_shared<Delegate<R, Args...>>(cb);
    }

    template <class R, typename... Args>
    Delegate<R, Args...> RegisterCallback(R (*f)(Args...))
    {
        auto cb = Delegate<R, Args...>(f);
        m_callBack = std::make_shared<Delegate<R, Args...>>(cb);
        return cb;
    }

    template <class R, class T, typename... Args>
    Delegate<R, Args...> RegisterCallback(R (T::*f)(Args...), T *t)
    {
        // std::function<R(Args...)> func = std::bind(f, t);// not work
        auto func = [t, f](Args... args)
        { return (t->*f)(std::forward<Args>(args)...); };
        auto cb = Delegate<R, Args...>(func);
        m_callBack = std::make_shared<Delegate<R, Args...>>(cb);
        return cb;
    }

    /*
     * @param R: Return type
     * @param ...args : notice the [number] and [type] of passed parameter must be defined clearly.
     *                  such as: the passed parameter should be defined with type 0.5f or 0.5d, instead of 0.5. 
     *                  otherwise the static_pointer_cast may get the wrong type. 
     */
    template <typename R = void, typename... Args>
    R InvokeCallback(Args &&...args)
    {
        //LOG_DEBUG() << "InvokeCallback" << m_callBack.get();
        if (!m_callBack)
            return R();
        std::shared_ptr<Delegate<R, Args...>> childPtr = std::static_pointer_cast<Delegate<R, Args...>>(m_callBack);
        if (childPtr)
        {
            //std::cout << "childPtr " << childPtr.get() << "m_callBack " << m_callBack.get() << std::endl;
            return childPtr->InvokeCallback(std::forward<Args>(args)...);
            // childPtr->InvokeCallback(args...);
        }
    }

protected:
    std::shared_ptr<DelegateBase> m_callBack = {};
};


END_NS_ZF

#endif // !ZF_GLOBAL_DELEGATE_H!