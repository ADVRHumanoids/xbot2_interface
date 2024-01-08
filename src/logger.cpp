#include <xbot2_interface/logger.h>

#define RT_LOG_RESET "\033[0m"
#define RT_LOG_BLACK "\033[30m"   /* Black */
#define RT_LOG_RED "\033[31m"     /* Red */
#define RT_LOG_GREEN "\033[32m"   /* Green */
#define RT_LOG_YELLOW "\033[33m"  /* Yellow */
#define RT_LOG_BLUE "\033[34m"    /* Blue */
#define RT_LOG_MAGENTA "\033[35m" /* Magenta */
#define RT_LOG_CYAN "\033[36m"    /* Cyan */
#define RT_LOG_WHITE "\033[37m"   /* White */

namespace XBot {
inline namespace v2 {

LoggerClass Logger::_logger("");

std::ostream &Logger::error(Logger::Severity s)
{
    return _logger.error(s);
}

void Logger::error(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    _logger.__error(Logger::Severity::HIGH, fmt, args);

    va_end(args);
}

void Logger::error(Logger::Severity s, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    _logger.__error(s, fmt, args);

    va_end(args);
}

void Logger::success(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    _logger.__success(Logger::Severity::LOW, fmt, args);

    va_end(args);
}

void Logger::success(Logger::Severity s, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    _logger.__success(s, fmt, args);

    va_end(args);
}

void Logger::warning(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    _logger.__warning(Logger::Severity::MID, fmt, args);

    va_end(args);
}

void Logger::warning(Logger::Severity s, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    _logger.__warning(s, fmt, args);

    va_end(args);
}

Endl &Logger::endl()
{
    return _logger.endl();
}

std::ostream &Logger::info(Logger::Severity s)
{
    return _logger.info(s);
}

void Logger::info(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    _logger.__info(Logger::Severity::LOW, fmt, args);

    va_end(args);
}

void Logger::info(Logger::Severity s, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    _logger.__info(s, fmt, args);

    va_end(args);
}

std::ostream &Logger::log()
{
    return _logger.log();
}

void Logger::SetVerbosityLevel(Logger::Severity s)
{
    _logger.setVerbosityLevel(s);
}

std::ostream &Logger::success(Logger::Severity s)
{
    return _logger.success(s);
}

std::ostream &Logger::warning(Logger::Severity s)
{
    return _logger.warning(s);
}

Logger::Severity Logger::GetVerbosityLevel()
{
    return _logger.getVerbosityLevel();
}

void Logger::SetOnPrintCallback(std::function<void(char *, int, Logger::Severity)> f)
{
    _logger.setOnPrintCallback(f);
}

std::ostream &bold_on(std::ostream &os)
{
    return os << "\e[1m";
}

std::ostream &bold_off(std::ostream &os)
{
    return os << "\e[0m";
}

std::ostream &color_green(std::ostream &os)
{
    return os << RT_LOG_GREEN;
}

std::ostream &color_red(std::ostream &os)
{
    return os << RT_LOG_RED;
}

std::ostream &color_yellow(std::ostream &os)
{
    return os << RT_LOG_YELLOW;
}

std::ostream &color_reset(std::ostream &os)
{
    return os << RT_LOG_RESET;
}

/* LoggerClass impl */

void LoggerClass::DefaultOnPrint(char *msg, int n_chars, Logger::Severity s)
{
    printf("%s", msg);
    fflush(stdout);
}

LoggerClass::LoggerClass(std::string name)
    : _endl(*this)
    , _name(name)
    , _verbosity_level(Logger::Severity::LOW)
    , _severity(Logger::Severity::LOW)
    , _on_print(&LoggerClass::DefaultOnPrint)
{
    if (_name != "") {
        _name_tag = " (" + name + ")";
    }

    _sink.open(_buffer);

    const char *xbot_verbose_env = std::getenv("XBOT_VERBOSE");
    if (xbot_verbose_env) {
        _verbosity_level = static_cast<Logger::Severity>(std::atoi(xbot_verbose_env));
    }
}

LoggerClass::LoggerClass(std::string logger_name,
                         std::function<void(char *, int, Logger::Severity)> f)
    : LoggerClass(logger_name)
{
    setOnPrintCallback(f);
}

XBot::LoggerClass::~LoggerClass()
{
    _sink.close();
}

void LoggerClass::init_sink()
{
    memset(_buffer, 0, BUFFER_SIZE);
    _sink.seekp(0);
}

void operator<<(std::ostream &os, Endl &endl)
{
    os << color_reset << "\n";
    endl.print();
}

void Endl::print()
{
    _logger_handle.print();
}

std::ostream &LoggerClass::log()
{
    if (_sink.tellp() == 0) {
        memset(_buffer, 0, BUFFER_SIZE);
    }

    return _sink;
}

std::ostream &LoggerClass::info(Logger::Severity s)
{
    _severity = s;

    init_sink();
    _sink << bold_on << "[info" << _name_tag << "] " << bold_off;
    return _sink;
};

void XBot::LoggerClass::__fmt_print(const char *fmt, va_list args)
{
    int pos = _sink.tellp();
    int nchars = vsnprintf(&_buffer[pos], (BUFFER_SIZE - pos), fmt, args);

    _sink.seekp(std::min(pos + nchars, BUFFER_SIZE - 1));

    _sink << color_reset;

    print();
}

void LoggerClass::info(Logger::Severity s, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    __info(s, fmt, args);

    va_end(args);
}

void LoggerClass::info(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    __info(Logger::Severity::LOW, fmt, args);

    va_end(args);
}

void XBot::LoggerClass::__info(Logger::Severity s, const char *fmt, va_list args)
{
    info(s);

    __fmt_print(fmt, args);
}

std::ostream &LoggerClass::error(Logger::Severity s)
{
    _severity = s;

    init_sink();
    _sink << bold_on << color_red << "[err " << _name_tag << "] " << bold_off << color_red;
    return _sink;
};

void LoggerClass::error(Logger::Severity s, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    __error(s, fmt, args);

    va_end(args);
}

void LoggerClass::error(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    __error(Logger::Severity::HIGH, fmt, args);

    va_end(args);
}

void XBot::LoggerClass::__error(Logger::Severity s, const char *fmt, va_list args)
{
    error(s);

    __fmt_print(fmt, args);
}

std::ostream &LoggerClass::warning(Logger::Severity s)
{
    _severity = s;

    init_sink();
    _sink << bold_on << color_yellow << "[warn" << _name_tag << "] " << bold_off << color_yellow;
    return _sink;
};

void LoggerClass::warning(Logger::Severity s, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    __warning(s, fmt, args);

    va_end(args);
}

void LoggerClass::warning(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    __warning(Logger::Severity::MID, fmt, args);

    va_end(args);
}

void XBot::LoggerClass::__warning(Logger::Severity s, const char *fmt, va_list args)
{
    warning(s);

    __fmt_print(fmt, args);
}

std::ostream &LoggerClass::success(Logger::Severity s)
{
    _severity = s;

    init_sink();
    _sink << bold_on << color_green << "[ok  " << _name_tag << "] " << bold_off << color_green;
    return _sink;
};

void LoggerClass::success(Logger::Severity s, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    __success(s, fmt, args);

    va_end(args);
}

void LoggerClass::success(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    __success(Logger::Severity::LOW, fmt, args);

    va_end(args);
}

void XBot::LoggerClass::__success(Logger::Severity s, const char *fmt, va_list args)
{
    success(s);

    __fmt_print(fmt, args);
}

Endl &LoggerClass::endl()
{
    return _endl;
}

void LoggerClass::setVerbosityLevel(Logger::Severity s)
{
    _verbosity_level = s;
}

Logger::Severity LoggerClass::getVerbosityLevel() const
{
    return _verbosity_level;
}

void LoggerClass::setOnPrintCallback(std::function<void(char *, int, Logger::Severity)> f)
{
    _on_print = f;
}

Endl::Endl(LoggerClass &logger_handle)
    : _logger_handle(logger_handle)
{}

void LoggerClass::print()
{
    if ((int) _severity >= (int) _verbosity_level) {
        if (_on_print) {
            _buffer[sizeof(_buffer) - 1] = '\0';
            int msg_size = std::min<int>(sizeof(_buffer) - 1, _sink.tellp());
            _on_print(_buffer, msg_size, _severity);
        }
    }

    _severity = Logger::Severity::HIGH;
}

} // namespace v2
} // namespace XBot
