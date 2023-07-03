#include "music_player.hpp"
#include "drivers.hpp"

namespace music{

MusicPlayer::MusicPlayer(src::Drivers *drivers, const vector<pair<float, float>>& score, unsigned tempo) :
    drivers(drivers), yourSong(score), tempo(tempo), index(0), resting(false), isFinished(false) {}

void MusicPlayer::execute() {
    if(!timeout.isExpired() || isFinished) return;
    if(index < yourSong.size()){
        if(!resting){
            tap::buzzer::playNote(&drivers->pwm, (yourSong[index]).first);
            timeout.restart(tempo * (yourSong[index]).second * 0.8f);
            resting = true;
        }
        else{
            tap::buzzer::playNote(&(drivers->pwm), 0);
            timeout.restart(tempo * (yourSong[index]).second * 0.2f);
            index++;
            resting = false;
        }
    }
    else{
        tap::buzzer::playNote(&drivers->pwm, 0);
        isFinished = true;
    }
    // for(size_t i = 0; i < yourSong.size(); i++){
    //     tap::buzzer::playNote(&drivers->pwm, (yourSong[i]).first);
    //     modm::delay_ms(tempo * (yourSong[i]).second * 0.8f);
    //     tap::buzzer::playNote(&(drivers->pwm), 0);
    //     modm::delay_ms(tempo * (yourSong[i]).second * 0.2f);
    // }
    // tap::buzzer::playNote(&(drivers->pwm), 0); //silences buzzer
    //drivers->leds.set(drivers->leds.Green, true);
}

//stops all sounds from the buzzer
void MusicPlayer::clearNote(){
    tap::buzzer::playNote(&(drivers->pwm), 0);
}

//plays a given note
void MusicPlayer::playGivenNote(float note){
    tap::buzzer::playNote(&(drivers->pwm), note);
}
}//namespace music
