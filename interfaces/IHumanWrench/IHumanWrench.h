/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef HDE_INTERFACES_IHUMANWRENCH
#define HDE_INTERFACES_IHUMANWRENCH

#include <string>
#include <vector>

namespace hde {
    namespace interfaces {
        class IHumanWrench;
    } // namespace interfaces
} // namespace hde

class hde::interfaces::IHumanWrench
{
public:
    virtual ~IHumanWrench() = default;

    enum class WrenchSourceType
    {
        Fixed,
        Robot,
        Dummy, // TODO
    };

    enum class WrenchReferenceFrame
    {
        Link,
        Centroidal,
        Base,
        World,
    };

    enum class WrenchType
    {
        Measured,
        Estimated,
    };

    // TODO: Check if the task type enum can be improved
    enum class TaskType
    {
        Task1,
        Task2,
    };

    virtual std::vector<std::pair<std::string, WrenchSourceType>> getWrenchSourceNameAndType() const = 0;
    virtual std::vector<std::string> getWrenchSourceNames() const = 0;
    virtual size_t getNumberOfWrenchSources() const = 0;

    virtual std::vector<double> getWrenches() const = 0;
    virtual std::vector<double> getWrenchesInFrame(TaskType, WrenchType, WrenchReferenceFrame) const = 0;

};

#endif // HDE_INTERFACES_IHUMANWRENCH
