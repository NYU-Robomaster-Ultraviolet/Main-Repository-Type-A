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
}

//stops all sounds from the buzzer
void MusicPlayer::clearNote(){
    tap::buzzer::playNote(&(drivers->pwm), 0);
}

//plays a given note
void MusicPlayer::playGivenNote(float note){
    tap::buzzer::playNote(&(drivers->pwm), note);
}
void MusicPlayer::resetSong() {
    resting = false;
    index = 0;
    isFinished = false;
    init();
}
    
void MusicPlayer::changeSong(const vector<pair<float, float>>& newScore, unsigned newTempo){
    yourSong = newScore;
    tempo = newTempo;
    resetSong();
}
}//namespace music
