#ifndef MUSIC_PLAYER_HPP_
#define MUSIC_PLAYER_HPP_

#include "drivers.hpp"
#include "vector"
#include "tap/communication/sensors/buzzer/buzzer.hpp"
#include "notes.hpp"
#include "music_scores/playlist.hpp"


using namespace tap;
using namespace tap::buzzer;

namespace music{

class MusicPlayer{
    public:
        MusicPlayer(src::Drivers *drivers, const vector<pair<float, float>>& score, unsigned tempo);

        void execute();

    private:
        src::Drivers* drivers;
        const vector<pair<float, float>> yourSong;
        const unsigned tempo;
};
} //namespace music

#endif