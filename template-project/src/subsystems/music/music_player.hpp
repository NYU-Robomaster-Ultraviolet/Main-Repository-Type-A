#ifndef MUSIC_PLAYER_HPP_
#define MUSIC_PLAYER_HPP_

#include "vector"
#include "tap/communication/sensors/buzzer/buzzer.hpp"
#include "notes.hpp"
#include "music_scores/playlist.hpp"
#include "tap/architecture/timeout.hpp"


using namespace tap;
using namespace tap::buzzer;
namespace src{
    class Drivers;
}

namespace music{

class MusicPlayer{
    public:
        /**
         * @brief Music player is an interface for the buzzer, allowing you to play a series of notes in a 
         * premade "music score" structure
         * @param drivers : pointer to the drivers class
         * @param score : a vector of two float pairs, indicating each note with its length
         * @param tempo : the bpm of the song
         */
        MusicPlayer(src::Drivers *drivers, const vector<pair<float, float>>& score, unsigned tempo);

        //retruns if the current song is finished
        bool finishedSong() const {return isFinished;}

        /**
         * @brief Changes the note being played by the buzzer depending on timouts and score given,
         * needs to be constantly called to maintain tempo
         */
        void execute();

        /**
         * @brief checks the tempo, indicating if the next note should be played or if the current note should end
         * 
         * @return true: not time to switch
         * @return false: time to switch
         */
        bool nextNote() const {return timeout.isExpired();}

        //silences the buzzer
        void clearNote();

        //restarts from the beginning of the song
        void resetSong();

        /**
         * \@brief Plays a given frequency
         * 
         * @param note : Frequency in HZ, examples of standard notes in notes.hpp
         */
        void playGivenNote(float note);

        //restarts the timer
        void init(){timeout.restart(1);}

        /**
         * @brief Changes the song and restarts from the beginning
         * 
         * @param newScore : a vector of pairs of floats indicating notes and length
         * @param newTempo : the new tempo of the song in bpm
         * @return ** void 
         */
        void changeSong(const vector<pair<float, float>>& newScore, unsigned newTempo);

    private:
        src::Drivers* drivers;
        vector<pair<float, float>> yourSong;
        unsigned tempo;
        size_t index;
        bool resting;
        bool isFinished;
        tap::arch::MilliTimeout timeout;
        
};
} //namespace music

#endif