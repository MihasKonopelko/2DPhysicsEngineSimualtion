#ifndef FIXTUREUSERDATACONTAINER_H
#define FIXTUREUSERDATACONTAINER_H
#include <string>

class FixtureUserDataContainer
{
    public:
        FixtureUserDataContainer(std::string name);
        virtual ~FixtureUserDataContainer();

        std::string GetName() { return m_name; }
    protected:
    private:
        std::string m_name;
};

#endif // FIXTUREUSERDATACONTAINER_H
