#ifndef CYBER_ANGEL_HPP_
#define CYBER_ANGEL_HPP_
#include "vector"
#include "subsystems/music/notes.hpp"

using namespace std;

namespace music{

//establishes tempo
static const unsigned CYBER_ANGEL_TEMPO = 120; //in ms this is = 60 BPM
static const unsigned CYBER_ANGEL_BPM = 60000 / CYBER_ANGEL_TEMPO; //60000 = 1 min in ms

/*this is the vector that contains your song Each element of the vector are floats
The first: containing the note to be played, see the noteshpp file to see the frequencies used
    For example, if you want to use the note C in the 3rd octave use O3C for your note
The second: represents the amount of time that a note should be played for. This value will be multiplied by
    your tempo, so a value of 1.0f will represent a quarter rest given 4/4 time.
Note: If you want a rest, put the note value as 0.0f
*/

const vector<pair<float, float>> CYBER_ANGEL{

//Cyber Angel - Hanser
//4/4 time

//measure 1
pair<float, float>(O4FS, NDE),
pair<float, float>(O4FS, NDE),
pair<float, float>(O4FS, NDE),  
pair<float, float>(O4F, NDE),
pair<float, float>(O4CS, NE),
pair<float, float>(O4F, NE),
//measure 2  
pair<float, float>(O4CS, NDE),
pair<float, float>(O3AS, NDE),  
pair<float, float>(O3GS, NDE),
pair<float, float>(O3AS, NDE),
pair<float, float>(O3GS, NE),
pair<float, float>(O3AS, NE),
//measure 3
pair<float, float>(O4CS, NDE),
pair<float, float>(O4FS, NDE),
pair<float, float>(O4F, NDE),
pair<float, float>(O4FS, NDE),
pair<float, float>(O4F, NE),
pair<float, float>(O4GS, NE),
//measure 4
pair<float, float>(O4GS, NDE),
pair<float, float>(O4GS, NDE),
pair<float, float>(O4GS, NDE),
pair<float, float>(O4AS, NDE),
pair<float, float>(O4FS, NE),
pair<float, float>(O4F, NE),
//measure 5
pair<float, float>(O4FS, NDE),
pair<float, float>(O4FS, NDE),
pair<float, float>(O4FS, NDE),  
pair<float, float>(O4F, NDE),
pair<float, float>(O4CS, NE),
pair<float, float>(O4F, NE),
//measure 6 
pair<float, float>(O4CS, NDE),
pair<float, float>(O3AS, NDE),  
pair<float, float>(O3GS, NDE),
pair<float, float>(O3AS, NDE),
pair<float, float>(O3GS, NE),
pair<float, float>(O3AS, NE),
//measure 7
pair<float, float>(O4CS, NDE),
pair<float, float>(O4FS, NDE),
pair<float, float>(O4F, NDE),
pair<float, float>(O4FS, NDE),
pair<float, float>(O4F, NE),
pair<float, float>(O4GS, NE),
//measure 8
pair<float, float>(O4GS, NDE),
pair<float, float>(O4GS, NDE),
pair<float, float>(O4GS, NDE),
pair<float, float>(O4AS, NDE),
pair<float, float>(O4FS, NE),
pair<float, float>(O4F, NE),
//measure 9: Second Meloady
pair<float, float>(O4FS, NDE),
pair<float, float>(O4FS, NDE),
pair<float, float>(O4CS, NE),
pair<float, float>(O4DS, NQ),
pair<float, float>(O4FS, NQ),
//measure 10
pair<float, float>(O4F, NDE),
pair<float, float>(O4FS, NDE),
pair<float, float>(O4GS, NE),
pair<float, float>(O4CS, NQ),
pair<float, float>(O4CS, NE),
pair<float, float>(O3AS, NE),
//measure 11
pair<float, float>(O3B, NDE),
pair<float, float>(O4FS, NDE),
pair<float, float>(O4GS, NE),
pair<float, float>(O4F, NDE),
pair<float, float>(O4CS, NDE),
pair<float, float>(O3GS, NE),
//measure 12
pair<float, float>(O3AS, NQ),
pair<float, float>(O4CS, NQ),
pair<float, float>(O4CS, NQ),
pair<float, float>(O4DS, NE),
pair<float, float>(O4F, NE),
//measure 13
pair<float, float>(O4FS, NDE),
pair<float, float>(O4F, NDE),
pair<float, float>(O4CS, NE),
pair<float, float>(O4DS, NQ),
pair<float, float>(O4AS, NQ),
//measure 14
pair<float, float>(O4FS, NDE),
pair<float, float>(O4CS, NDE),
pair<float, float>(O4GS, NE),
pair<float, float>(O4AS, NQ),
pair<float, float>(O4DS, NE),
pair<float, float>(O4FS, NE),
//measure 15
pair<float, float>(O4AS, NDE), 
pair<float, float>(O3B, NDE), 
pair<float, float>(O4AS, NDE),
pair<float, float>(O4GS, NDE),
pair<float, float>(O4FS, NDE),
pair<float, float>(O4F, NE),
//measure 16
pair<float, float>(O4F, NE + NDH),
pair<float, float>(O4DS, NE),
//measure 17
pair<float, float>(O4DS, NH),
pair<float, float>(0, NQ),
pair<float, float>(O4DS, NE),
pair<float, float>(O4F, NE),
//measure 18: melody 3 New Key Signature
pair<float, float>(O4G, NQ),
pair<float, float>(O5C, NQ), //change from C
pair<float, float>(O4AS, NDE),
pair<float, float>(O4G, NDE),
pair<float, float>(O4F, NE),
//measure 19
pair<float, float>(O4G, NDE),
pair<float, float>(O4G, NDE),
pair<float, float>(O4CS, NS),
pair<float, float>(O4F, NDQ),
pair<float, float>(O4C, NE),
pair<float, float>(O4CS, NE),
//measure 20
pair<float, float>(O4G, NDE),
pair<float, float>(O4C, NDE),
pair<float, float>(O4G, NE),
pair<float, float>(O4F, NQ),
pair<float, float>(O3AS, NE),
pair<float, float>(O4F, NE),
//measure 21
pair<float, float>(O4GS, NDE),
pair<float, float>(O4AS, NDE),
pair<float, float>(O4F, NE),
pair<float, float>(O4G, NQ),
pair<float, float>(O4DS, NE),
pair<float, float>(O4F, NE),
//measure 22
pair<float, float>(O4G, NQ),
pair<float, float>(O5C, NQ),
pair<float, float>(O4AS, NDE),
pair<float, float>(O4G, NDE),
pair<float, float>(O4F, NE),
//measure 23
pair<float, float>(O4G, NDE),
pair<float, float>(O4G, NDE),
pair<float, float>(O4CS, NS),
pair<float, float>(O4F, NDQ),
pair<float, float>(O4C, NE),
pair<float, float>(O4CS, NE),
//measure 24
pair<float, float>(O4G, NDE),
pair<float, float>(O4CS, NDE),
pair<float, float>(O4G, NE),
pair<float, float>(O4F, NE),
pair<float, float>(O4G, NE),
pair<float, float>(O4AS, NE),
pair<float, float>(O5D, NE),
//measure 25
pair<float, float>(O5DS, NDH),


};//yourSong
};//namespace music

#endif