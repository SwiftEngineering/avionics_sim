/**
 * @brief       Property class. This class removes the need for set/get methods without violating concept of encapsulation.
 * Adapted from https://stackoverflow.com/questions/3632533/set-get-methods-in-c
 * @file        Property.hpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2020, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

template <class T>
class DefaultPredicate
{
public:
  static bool CheckSetter (T value)
  {
    return true;
  }
  static void CheckGetter (T value)
  {
  }
};

template <class T, class Predicate = DefaultPredicate <T>>
class Property
{
public:
  operator T ()
  {
    Predicate::CheckGetter (m_storage);
    return m_storage;
  }
  Property <T, Predicate> &operator = (T rhs)
  {
    if (Predicate::CheckSetter (rhs))
    {
      m_storage = rhs;
    }
    return *this;
  }
private:
  T m_storage;
};
