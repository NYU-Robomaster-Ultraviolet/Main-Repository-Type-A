#ifndef NEVER_SURRENDER_HPP_
#define NEVER_SURRENDER_HPP_
#include "vector"
#include "subsystems/music/notes.hpp"

using namespace std;

namespace music{

//establishes tempo
static const unsigned NEVER_SURRENDER_TEMPO = 166; //in ms this is = 60 BPM
static const unsigned NEVER_SURRENDER_BPM = 60000 / NEVER_SURRENDER_TEMPO; //60000 = 1 min in ms

/*this is the vector that contains your song Each element of the vector are floats
The first: containing the note to be played, see the noteshpp file to see the frequencies used
    For example, if you want to use the note C in the 3rd octave use O3C for your note
The second: represents the amount of time that a note should be played for. This value will be multiplied by
    your tempo, so a value of 1.0f will represent a quarter rest given 4/4 time.
Note: If you want a rest, put the note value as 0.0f
*/

const vector<pair<float, float>> NEVER_SURRENDER{
    
//Never Surrender - Mizuki Nana
//4/4 time

//measure 1
pair<float, float>(O5C, NE),
pair<float, float>(O4G, NE),
pair<float, float>(O4E, NE),  
pair<float, float>(O4B, NE),
pair<float, float>(O4G, NE),  
pair<float, float>(O4D, NE),
pair<float, float>(O4G, NE),  
pair<float, float>(O4D, NE),

//measure 2
pair<float, float>(O3B, NE),  
pair<float, float>(O4E, NDQ),
pair<float, float>(0, NH),

//measure 3
pair<float, float>(O5C, NE),
pair<float, float>(O4G, NE),
pair<float, float>(O4E, NE),  
pair<float, float>(O4B, NE),
pair<float, float>(O4G, NE),  
pair<float, float>(O4D, NE),
pair<float, float>(O4G, NE),  
pair<float, float>(O4D, NE),

//measure 4
pair<float, float>(O3B, NE),  
pair<float, float>(O4E, NDQ),
pair<float, float>(0, NH),

//measure 5
pair<float, float>(O4G, NDQ),  
pair<float, float>(O4F, NDQ),
pair<float, float>(O4E, NQ), 

//measure 6 
pair<float, float>(O4C, NDQ),
pair<float, float>(O4D, NDQ),  
pair<float, float>(O3A, NQ),

//measure 7
pair<float, float>(O4E, NH),  
pair<float, float>(0, NE),
pair<float, float>(O3A, NE),  
pair<float, float>(O4E, NE),
pair<float, float>(O4A, NE), 

//measure 8
pair<float, float>(O5E, NW),

//measure 9
pair<float, float>(0, NDQ),  
pair<float, float>(O3DS, NE),
pair<float, float>(O3E, NE),  
pair<float, float>(O3A, NE),
pair<float, float>(O4E, NE), 

pair<float, float>(O4D, NDQ), //measure 10 
pair<float, float>(O4C, NE),  
pair<float, float>(O3B, NQ),
pair<float, float>(O3A, NQ), 

pair<float, float>(O3B, NQ), //measure 11
pair<float, float>(O4C, NE),  
pair<float, float>(O4D, NE),
pair<float, float>(O3B, NQ),  
pair<float, float>(O3G, NE),
pair<float, float>(O3D, NE),

pair<float, float>(O3E, NDQ), //measure 12
pair<float, float>(0, NE),  
pair<float, float>(O4A, NS),
pair<float, float>(O4B, NS),  
pair<float, float>(O5C, NE),
pair<float, float>(O4B, NE),  
pair<float, float>(O4G, NE),

pair<float, float>(O4D, NQ), //measure 13
pair<float, float>(O4C, NE),
pair<float, float>(O3B, NS),  
pair<float, float>(0, NS),
pair<float, float>(O3E, NE),  
pair<float, float>(O3G, NE),
pair<float, float>(O3B, NE),  
pair<float, float>(O4E, NE),

pair<float, float>(O4G, NDQ), //measure 14
pair<float, float>(O4F, NE),
pair<float, float>(O4E, NQ),  
pair<float, float>(O4F, NE),
pair<float, float>(O4E, NE), 

pair<float, float>(O4D, NDQ), //measure 15
pair<float, float>(O3G, NE),  
pair<float, float>(O4D, NQ),
pair<float, float>(O3B, NE),

pair<float, float>(O3G, NE + NH), //measure 16
pair<float, float>(O3G, NS),  
pair<float, float>(O3A, NS),
pair<float, float>(O3B, NS), 
pair<float, float>(O4C, NS), 
pair<float, float>(O4D, NS),  
pair<float, float>(O4E, NS),

pair<float, float>(O4F, NQ), //measure 17 
pair<float, float>(O4E, NE), 
pair<float, float>(O4G, NE),  
pair<float, float>(O4F, NQ), 
pair<float, float>(O3B, NE),
pair<float, float>(O4D, NE), 
pair<float, float>(O4F, NE), 

//measure 18
pair<float, float>(O4E, NQ), 
pair<float, float>(0, NE), 
pair<float, float>(O4F, NQ), 
pair<float, float>(0, NE), 
pair<float, float>(O4G, NQ), 

//measure 19 (2/4)
pair<float, float>(0, NE), 
pair<float, float>(O4GS, NQ), 
pair<float, float>(0, NE),

//measure 20
pair<float, float>(O4G, NS),
pair<float, float>(O4FS, NS),
pair<float, float>(O4F, NS),
pair<float, float>(O4E, NS),
pair<float, float>(O4DS, NS),
pair<float, float>(O4D, NS),
pair<float, float>(O4CS, NS),
pair<float, float>(O4C, NS),
pair<float, float>(O3B, NS),
pair<float, float>(O3AS, NS),
pair<float, float>(O3A, NS),
pair<float, float>(O3GS, NS),
pair<float, float>(O3G, NS),
pair<float, float>(O3FS, NS),
pair<float, float>(O3F, NS),
pair<float, float>(O3E, NS)
//measure 21
};//yourSong
};//namespace music

#endif