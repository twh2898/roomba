#include "robbie/Config.hpp"

#include <fstream>

namespace robbie {
    using std::fstream;

    Config::Config() {}

    Config Config::fromFile(const string & file) {
        fstream f(file);
        json config = json::parse(f);

        // TODO: parse structure
    }
}
