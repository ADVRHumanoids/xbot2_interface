#ifndef XBOT2IFC_CONFIG_OPTIONS_HXX
#define XBOT2IFC_CONFIG_OPTIONS_HXX

template<typename ParameterType>
bool ConfigOptions::get_parameter(std::string key, ParameterType &value) const
{
    auto it = params.find(key);

    if(it == params.end())
    {
        return false;
    }

    value = std::any_cast<ParameterType>(it->second);

    return true;
}

template<typename ParameterType>
bool ConfigOptions::set_parameter(std::string key, const ParameterType &value)
{
    params[key] = value;

    return true;
}

#endif // CONFIG_OPTIONS_HXX
