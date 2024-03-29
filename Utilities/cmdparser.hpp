// The application is licensed under the MIT License (MIT).

// Copyright (c) 2015 - 2016 Florian Rappl

// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
// associated documentation files (the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge, publish, distribute,
// sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all copies or
// substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
// NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

/*
  This file is part of the C++ CmdParser utility.
  Copyright (c) 2015 - 2019 Florian Rappl

  Github repository: https://github.com/FlorianRappl/CmdParser
*/

#pragma once
#include <functional>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cli {
/// Class used to wrap integer types to specify desired numerical base for specific argument parsing
template<typename T, int numericalBase = 0>
class NumericalBase
{
  public:
    /// This constructor required for correct AgrumentCountChecker initialization
    NumericalBase()
        : value(0)
        , base(numericalBase)
    {
    }

    /// This constructor required for default value initialization
    /// \param val comes from default value
    NumericalBase(T val)
        : value(val)
        , base(numericalBase)
    {
    }

    operator T() const { return this->value; }
    operator T*() { return this->value; }

    T value;
    unsigned int base;
};

struct CallbackArgs
{
    const std::vector<std::string>& arguments;
    std::ostream& output;
    std::ostream& error;
};
class Parser
{
  private:
    class CmdBase
    {
      public:
        explicit CmdBase(const std::string& name,
                         const std::string& alternative,
                         std::string  description,
                         bool required,
                         bool dominant,
                         bool variadic)
            : name(name)
            , command(!name.empty() ? "-" + name : "")
            , alternative(!alternative.empty() ? "--" + alternative : "")
            , description(std::move(description))
            , required(required)
            , 
             arguments({})
            , dominant(dominant)
            , variadic(variadic)
        {
        }

        virtual ~CmdBase() = default;

        std::string name;
        std::string command;
        std::string alternative;
        std::string description;
        bool required;
        bool handled{false};
        std::vector<std::string> arguments;
        bool const dominant;
        bool const variadic;

        [[nodiscard]] virtual auto print_value() const -> std::string = 0;
        virtual auto parse(std::ostream& output, std::ostream& error) -> bool = 0;

        [[nodiscard]] auto is(const std::string& given) const -> bool { return given == command || given == alternative; }
    };

    template<typename T>
    struct ArgumentCountChecker
    {
        static constexpr bool Variadic = false;
    };

    template<typename T>
    struct ArgumentCountChecker<cli::NumericalBase<T>>
    {
        static constexpr bool Variadic = false;
    };

    template<typename T>
    struct ArgumentCountChecker<std::vector<T>>
    {
        static constexpr bool Variadic = true;
    };

    template<typename T>
    class CmdFunction final : public CmdBase
    {
      public:
        explicit CmdFunction(const std::string& name,
                             const std::string& alternative,
                             const std::string& description,
                             bool required,
                             bool dominant)
            : CmdBase(name,
                      alternative,
                      description,
                      required,
                      dominant,
                      ArgumentCountChecker<T>::Variadic)
        {
        }

        auto parse(std::ostream& output, std::ostream& error) -> bool override
        {
            try {
                CallbackArgs args{ arguments, output, error };
                value = callback(args);
                return true;
            } catch (...) {
                return false;
            }
        }

        [[nodiscard]] auto print_value() const -> std::string override { return ""; }

        std::function<T(CallbackArgs&)> callback;
        T value;
    };

    template<typename T>
    class CmdArgument final : public CmdBase
    {
      public:
        explicit CmdArgument(const std::string& name,
                             const std::string& alternative,
                             const std::string& description,
                             bool required,
                             bool dominant)
            : CmdBase(name,
                      alternative,
                      description,
                      required,
                      dominant,
                      ArgumentCountChecker<T>::Variadic)
        {
        }

        auto parse(std::ostream&, std::ostream&) -> bool override
        {
            try {
                value = Parser::parse(arguments, value);
                return true;
            } catch (...) {
                return false;
            }
        }

        [[nodiscard]] auto print_value() const -> std::string override { return stringify(value); }

        T value;
    };

    static auto parse(const std::vector<std::string>& elements, const int&, int numberBase = 0) -> int
    {
        if (elements.size() != 1) {
            throw std::bad_cast();
}

        return std::stoi(elements[0], nullptr, numberBase);
    }

    static auto parse(const std::vector<std::string>& elements, const bool& defval) -> bool
    {
        if (!elements.empty()) {
            throw std::runtime_error("A boolean command line parameter cannot have any arguments.");
}

        return !defval;
    }

    static auto parse(const std::vector<std::string>& elements, const double&) -> double
    {
        if (elements.size() != 1) {
            throw std::bad_cast();
}

        return std::stod(elements[0]);
    }

    static auto parse(const std::vector<std::string>& elements, const float&) -> float
    {
        if (elements.size() != 1) {
            throw std::bad_cast();
}

        return std::stof(elements[0]);
    }

    static auto parse(const std::vector<std::string>& elements, const long double&) -> long double
    {
        if (elements.size() != 1) {
            throw std::bad_cast();
}

        return std::stold(elements[0]);
    }

    static auto parse(const std::vector<std::string>& elements,
                              const unsigned int&,
                              int numberBase = 0) -> unsigned int
    {
        if (elements.size() != 1) {
            throw std::bad_cast();
}

        return static_cast<unsigned int>(std::stoul(elements[0], nullptr, numberBase));
    }

    static auto parse(const std::vector<std::string>& elements,
                               const unsigned long&,
                               int numberBase = 0) -> unsigned long
    {
        if (elements.size() != 1) {
            throw std::bad_cast();
}

        return std::stoul(elements[0], nullptr, numberBase);
    }

    static auto parse(const std::vector<std::string>& elements,
                                    const unsigned long long&,
                                    int numberBase = 0) -> unsigned long long
    {
        if (elements.size() != 1) {
            throw std::bad_cast();
}

        return std::stoull(elements[0], nullptr, numberBase);
    }

    static auto parse(const std::vector<std::string>& elements,
                           const long long&,
                           int numberBase = 0) -> long long
    {
        if (elements.size() != 1) {
            throw std::bad_cast();
}

        return std::stoll(elements[0], nullptr, numberBase);
    }

    static auto parse(const std::vector<std::string>& elements, const long&, int numberBase = 0) -> long
    {
        if (elements.size() != 1) {
            throw std::bad_cast();
}

        return std::stol(elements[0], nullptr, numberBase);
    }

    static auto parse(const std::vector<std::string>& elements, const std::string&) -> std::string
    {
        if (elements.size() != 1) {
            throw std::bad_cast();
}

        return elements[0];
    }

    template<class T>
    static auto parse(const std::vector<std::string>& elements, const std::vector<T>&) -> std::vector<T>
    {
        const T defval = T();
        std::vector<T> values{};
        std::vector<std::string> buffer(1);

        for (const auto& element : elements) {
            buffer[0] = element;
            values.push_back(parse(buffer, defval));
        }

        return values;
    }

    template<typename T>
    static auto parse(const std::vector<std::string>& elements, const NumericalBase<T>& wrapper) -> T
    {
        return parse(elements, wrapper.value, 0);
    }

    /// Specialization for number wrapped into numerical base
    /// \tparam T base type of the argument
    /// \tparam base numerical base
    /// \param elements
    /// \param wrapper
    /// \return parsed number
    template<typename T, int base>
    static auto parse(const std::vector<std::string>& elements, const NumericalBase<T, base>& wrapper) -> T
    {
        return parse(elements, wrapper.value, wrapper.base);
    }

    template<class T>
    static auto stringify(const T& value) -> std::string
    {
        return std::to_string(value);
    }

    template<class T, int base>
    static auto stringify(const NumericalBase<T, base>& wrapper) -> std::string
    {
        return std::to_string(wrapper.value);
    }

    template<class T>
    static auto stringify(const std::vector<T>& values) -> std::string
    {
        std::stringstream ss{};
        ss << "[ ";

        for (const auto& value : values) {
            ss << stringify(value) << " ";
        }

        ss << "]";
        return ss.str();
    }

    static auto stringify(const std::string& str) -> std::string { return str; }

  public:
    explicit Parser(int argc, const char** argv)
        : _appname(argv[0])
    {
        for (int i = 1; i < argc; ++i) {
            _arguments.emplace_back(argv[i]);
        }
        enable_help();
    }

    explicit Parser(int argc, char** argv)
        : _appname(argv[0])
    {
        for (int i = 1; i < argc; ++i) {
            _arguments.emplace_back(argv[i]);
        }
        enable_help();
    }

    Parser(int argc, const char** argv, std::string generalProgramDescriptionForHelpText)
        : _appname(argv[0])
        , _general_help_text(std::move(generalProgramDescriptionForHelpText))
    {
        for (int i = 1; i < argc; ++i) {
            _arguments.emplace_back(argv[i]);
        }
        enable_help();
    }

    Parser(int argc, char** argv, std::string generalProgramDescriptionForHelpText)
        : _appname(argv[0])
        , _general_help_text(std::move(generalProgramDescriptionForHelpText))
    {
        for (int i = 1; i < argc; ++i) {
            _arguments.emplace_back(argv[i]);
        }
        enable_help();
    }

    ~Parser()
    {
        for (size_t i = 0, n = _commands.size(); i < n; ++i) {
            delete _commands[i];
        }
    }

    [[nodiscard]] auto has_help() const -> bool
    {
        for (const auto& command : _commands) {
            if (command->name == "h" && command->alternative == "--help") {
                return true;
            }
        }

        return false;
    }

    void enable_help()
    {
        set_callback("h",
                     "help",
                     std::function<bool(CallbackArgs&)>([this](CallbackArgs& args) {
                         args.output << this->usage();
#pragma warning(push)
#pragma warning(disable : 4702)
                         exit(0);
                         return false;
#pragma warning(pop)
                     }),
                     "",
                     true);
    }

    void disable_help()
    {
        for (auto command = _commands.begin(); command != _commands.end(); ++command) {
            if ((*command)->name == "h" && (*command)->alternative == "--help") {
                _commands.erase(command);
                break;
            }
        }
    }

    template<typename T>
    void set_default(bool is_required, const std::string& description = "")
    {
        auto command = new CmdArgument<T>{ "", "", description, is_required, false };
        _commands.push_back(command);
    }

    template<typename T>
    void set_required(const std::string& name,
                      const std::string& alternative,
                      const std::string& description = "",
                      bool dominant = false)
    {
        auto command = new CmdArgument<T>{ name, alternative, description, true, dominant };
        _commands.push_back(command);
    }

    template<typename T>
    void set_optional(const std::string& name,
                      const std::string& alternative,
                      T defaultValue,
                      const std::string& description = "",
                      bool dominant = false)
    {
        auto command = new CmdArgument<T>{ name, alternative, description, false, dominant };
        command->value = defaultValue;
        _commands.push_back(command);
    }

    template<typename T>
    void set_callback(const std::string& name,
                      const std::string& alternative,
                      std::function<T(CallbackArgs&)> callback,
                      const std::string& description = "",
                      bool dominant = false)
    {
        auto command = new CmdFunction<T>{ name, alternative, description, false, dominant };
        command->callback = callback;
        _commands.push_back(command);
    }

    inline void run_and_exit_if_error()
    {
        if (run() == false) {
            exit(1);
        }
    }

    inline auto run() -> bool { return run(std::cout, std::cerr); }

    inline auto run(std::ostream& output) -> bool { return run(output, std::cerr); }

    auto doesArgumentExist(const std::string& name, const std::string& altName) -> bool
    {
        for (const auto& argument : _arguments) {

            if (argument == '-' + name || argument == altName) {
                return true;
            }
        }

        return false;
    }

    inline auto doesHelpExist() -> bool { return doesArgumentExist("h", "--help"); }

    auto run(std::ostream& output, std::ostream& error) -> bool
    {
        if (!_arguments.empty()) {
            auto current = find_default();

            for (size_t i = 0, n = _arguments.size(); i < n; ++i) {
                auto isarg = !_arguments[i].empty() && _arguments[i][0] == '-';
                auto associated = isarg ? find(_arguments[i]) : nullptr;

                if (associated != nullptr) {
                    current = associated;
                    associated->handled = true;
                } else if (current == nullptr) {
                    error << no_default();
                    return false;
                } else {
                    current->arguments.push_back(_arguments[i]);
                    current->handled = true;
                    if (!current->variadic) {
                        // If the current command is not variadic, then no more arguments
                        // should be added to it. In this case, switch back to the default
                        // command.
                        current = find_default();
                    }
                }
            }
        }

        // First, parse dominant arguments since they succeed even if required
        // arguments are missing.
        for (auto command : _commands) {
            if (command->handled && command->dominant && !command->parse(output, error)) {
                error << howto_use(command);
                return false;
            }
        }

        // Next, check for any missing arguments.
        for (auto command : _commands) {
            if (command->required && !command->handled) {
                error << howto_required(command);
                return false;
            }
        }

        // Finally, parse all remaining arguments.
        for (auto command : _commands) {
            if (command->handled && !command->dominant && !command->parse(output, error)) {
                error << howto_use(command);
                return false;
            }
        }

        return true;
    }

    template<typename T>
    [[nodiscard]] [[nodiscard]] auto get(const std::string& name) const -> T
    {
        for (const auto& command : _commands) {
            if (command->name == name) {
                auto cmd = dynamic_cast<CmdArgument<T>*>(command);

                if (cmd == nullptr) {
                    throw std::runtime_error("Invalid usage of the parameter " + name +
                                             " detected.");
                }

                return cmd->value;
            }
        }

        throw std::runtime_error("The parameter " + name + " could not be found.");
    }

    template<typename T>
    auto get_if(const std::string& name, std::function<T(T)> callback) const -> T
    {
        auto value = get<T>(name);
        return callback(value);
    }

    [[nodiscard]] auto requirements() const -> int
    {
        int count = 0;

        for (const auto& command : _commands) {
            if (command->required) {
                ++count;
            }
        }

        return count;
    }

    [[nodiscard]] auto commands() const -> int { return static_cast<int>(_commands.size()); }

    [[nodiscard]] inline auto app_name() const -> const std::string& { return _appname; }

  protected:
    auto find(const std::string& name) -> CmdBase*
    {
        for (auto command : _commands) {
            if (command->is(name)) {
                return command;
            }
        }

        return nullptr;
    }

    auto find_default() -> CmdBase*
    {
        for (auto command : _commands) {
            if (command->name.empty()) {
                return command;
            }
        }

        return nullptr;
    }

    [[nodiscard]] auto usage() const -> std::string
    {
        std::stringstream ss{};
        ss << _general_help_text << "\n\n";
        ss << "Available parameters:\n\n";

        for (const auto& command : _commands) {
            ss << "  " << command->command << "\t" << command->alternative;

            if (command->required == true) {
                ss << "\t(required)";
            }

            ss << "\n   " << command->description;

            if (command->required == false) {
                ss << "\n   "
                   << "This parameter is optional. The default value is '" + command->print_value()
                   << "'.";
            }

            ss << "\n\n";
        }

        return ss.str();
    }

    void print_help(std::stringstream& ss) const
    {
        if (has_help()) {
            ss << "For more help use --help or -h.\n";
        }
    }

    auto howto_required(CmdBase* command) const -> std::string
    {
        std::stringstream ss{};
        ss << "The parameter " << command->name << " is required.\n";
        ss << command->description << '\n';
        print_help(ss);
        return ss.str();
    }

    auto howto_use(CmdBase* command) const -> std::string
    {
        std::stringstream ss{};
        ss << "The parameter " << command->name << " has invalid arguments.\n";
        ss << command->description << '\n';
        print_help(ss);
        return ss.str();
    }

    [[nodiscard]] auto no_default() const -> std::string
    {
        std::stringstream ss{};
        ss << "No default parameter has been specified.\n";
        ss << "The given argument must be used with a parameter.\n";
        print_help(ss);
        return ss.str();
    }

    [[nodiscard]] auto get_general_help_text() const -> const std::string& { return _general_help_text; }

    void set_general_help_text(const std::string& generalHelpText)
    {
        _general_help_text = generalHelpText;
    }

  private:
    const std::string _appname;
    std::string _general_help_text;
    std::vector<std::string> _arguments;
    std::vector<CmdBase*> _commands;
};
}