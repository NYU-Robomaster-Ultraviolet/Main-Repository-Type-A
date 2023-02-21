#include "music_player.hpp"

namespace music{

MusicPlayer::MusicPlayer(src::Drivers *drivers, const vector<pair<float, float>>& score, unsigned tempo) :
    drivers(drivers), yourSong(score), tempo(tempo) {}

void MusicPlayer::execute() {
    drivers->leds.set(drivers->leds.Red, yourSong.size() < 1);
    drivers->leds.set(drivers->leds.Green, yourSong.size() > 1);
    for(size_t i = 0; i < yourSong.size(); i++){
        tap::buzzer::playNote(&drivers->pwm, (yourSong[i]).first);
        modm::delay_ms(tempo * (yourSong[i]).second * 0.8f);
        tap::buzzer::playNote(&(drivers->pwm), 0);
        modm::delay_ms(tempo * (yourSong[i]).second * 0.2f);
    }
    tap::buzzer::playNote(&(drivers->pwm), 0); //silences buzzer
    //drivers->leds.set(drivers->leds.Green, true);
}
}//namespace music
