#ifndef PIANO_MAN_HPP_
#define PIANO_MAN_HPP_
#include "vector"
#include "subsystems/music/notes.hpp"

using namespace std;

namespace music{

//establishes tempo
static const unsigned PIANO_MAN_TEMPO = 180; //in ms this is = 60 BPM
static const unsigned PIANO_MAN_BPM = 60000 / PIANO_MAN_TEMPO; //60000 = 1 min in ms

/*this is the vector that contains your song Each element of the vector are floats
The first: containing the note to be played, see the noteshpp file to see the frequencies used
    For example, if you want to use the note C in the 3rd octave use O3C for your note
The second: represents the amount of time that a note should be played for. This value will be multiplied by
    your tempo, so a value of 1.0f will represent a quarter rest given 4/4 time.
Note: If you want a rest, put the note value as 0.0f
*/

const vector<pair<float, float>> PIANO_MAN{

    //Piano Man - Billy Joel 
    //3/4 time

//measure 79
pair<float, float>(O3G, NE),
pair<float, float>(O3G, NH),
pair<float, float>(O3G, NE),
//measure 80
pair<float, float>(O3G, NQ),
pair<float, float>(O3F, NDQ),
pair<float, float>(O3E, NE),
//measure 81
pair<float, float>(O3F, NE),
pair<float, float>(O3E, NQ),
pair<float, float>(O3C, NDQ + NDH), //measure 82
//measure 83
pair<float, float>(O2A, NQ), 
pair<float, float>(O3C, NDQ),
pair<float, float>(O3C, NE),
//measure 84
pair<float, float>(O3C, NH),
pair<float, float>(O3D, NQ),
//measure 85
pair<float, float>(O3D, NDH + NQ), //measure 86
pair<float, float>(O3E, NQ),
pair<float, float>(O3F, NQ),
//measure 87
pair<float, float>(O3G, NQ),
pair<float, float>(O3G, NDQ),
pair<float, float>(O3G, NE),
//measure 88
pair<float, float>(O3G, NQ),
pair<float, float>(O3F, NDQ),
pair<float, float>(O3F, NE),
//measure 89
pair<float, float>(O3F, NE),
pair<float, float>(O3E, NE),
pair<float, float>(O3C, NH),
//measure 90
pair<float, float>(0, NH),
pair<float, float>(O2G, NQ),
//measure 91
pair<float, float>(O2A, NQ),
pair<float, float>(O3C, NQ),
pair<float, float>(O3E, NQ),
//measure 92
pair<float, float>(O3F, NE),
pair<float, float>(O3E, NQ),
pair<float, float>(0, NE),
pair<float, float>(O3D, NQ),
//measure 93
pair<float, float>(O3C, NDH),
//measure 94
pair<float, float>(0, NH),
pair<float, float>(O4E, NE),
pair<float, float>(O3F, NE),
//measure 95
pair<float, float>(O5C, NH + NE),
pair<float, float>(O4A, NE),
//measure 96
pair<float, float>(O5C, NH),
pair<float, float>(O4A, NE),
pair<float, float>(O4G, NE),
//measure 97
pair<float, float>(O4A, NE),
pair<float, float>(O4G, NH + NE),
//measure 98
pair<float, float>(0, NDH),
//measure 99
pair<float, float>(O4C, NDH),
//measure 100
pair<float, float>(O4F, NE),
pair<float, float>(O4E, NQ + NE),
pair<float, float>(O4D, NQ),
//measure 101
pair<float, float>(O4C, NDH + NDH) //measure 102
};//yourSong
};//namespace music

#endif