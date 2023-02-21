#ifndef NOTES_HPP_
#define NOTES_HPP_

namespace music{
//these are the frequencies of common notes sorted by octave in a struct
//Notes range from A to G with octaves starting with C and ending with B from lowest to highest
//These notes use the treble clef and only include sharps (no flats)
//Piano Octaves range from Octave 1 to Octave 6 with some notes going into the edge Octaves.
//Note: S = Sharp

//Octave 0
static constexpr float O0C = 16.35f, O0CS = 17.32f, O0D = 18.35f, O0DS = 19.45f, O0E = 20.60f,
    O0F = 21.83f, O0FS = 23.12f, O0G = 24.5f, O0GS = 25.96f, O0A = 27.5f, O0AS = 29.14f, O0B = 30.87f;

//Octave 1
    static constexpr float O1C = 32.7f, O1CS = 34.65f, O1D = 36.71f, O1DS = 38.89f, O1E = 41.2f,
    O1F = 43.65f, O1FS = 46.25f, O1G = 49.0f, O1GS = 51.91f, O1A = 55.0f, O1AS = 58.27f, O1B = 61.74f;

//Octave 2
    static constexpr float O2C = 65.41f, O2CS = 69.30f, O2D = 73.42f, O2DS = 77.78f, O2E = 82.41f,
    O2F = 87.31f, O2FS = 92.5f, O2G = 98.0f, O2GS = 103.83f, O2A = 110.0f, O2AS = 116.54f, O2B = 123.47f;

//Octave 3
    static constexpr float O3C = 130.81f, O3CS = 138.59f, O3D = 146.83f, O3DS = 155.56f, O3E = 164.81f,
    O3F = 174.61f, O3FS = 185.0f, O3G = 196.0f, O3GS = 207.65f, O3A = 220.0f, O3AS = 233.08f, O3B = 246.94f;


//Octave 4
    static constexpr float O4C = 261.63f, O4CS = 277.18f, O4D = 293.66f, O4DS = 311.13f, O4E = 329.63f,
    O4F = 349.23f, O4FS = 369.99f, O4G = 392.0f, O4GS = 415.3f, O4A = 440.0f, O4AS = 466.16f, O4B = 493.88f;

//Octave 5
    static constexpr float O5C = 523.25f, O5CS = 554.37f, O5D = 587.33f, O5DS = 622.25f, O5E = 659.25f,
    o5F = 698.46f, O5FS = 739.99f, O5G = 783.99f, O5GS = 830.61f, O5A = 880.0f, O5AS = 932.33f, O5B = 987.77f;

//Octave 6
    static constexpr float O6C = 1046.5f, O6CS = 1108.73f, O6D = 1174.66f, O6DS = 1244.51f, O6E = 1318.51f,
    O6F = 1396.91f, O6FS = 1479.98f, O6G = 1567.98f, O6GS = 1661.22f, O6A = 1760.0f, O6AS = 1864.66f, O6B = 1975.53f;

//Octave 7
    static constexpr float O7C = 2093.0f, O7CS = 2217.46f, O7D = 2349.32f, O7DS = 2489.02f, O7E = 2637.02f,
    O7F = 2793.83f, O7FS = 2959.96f, O7G = 3135.96f, O7GS = 3322.44f, O7A = 3520.0f, O7AS = 3729.31f, O7B = 3951.07f;

//Note values
    //Integer note values 

    //quarter note
    static constexpr float NQ = 1.0f;
    //half note
    static constexpr float NH = 2.0f;
    //doted half note
    static constexpr float NDH = 3.0f;
    //whole note
    static constexpr float NW = 4.0f;

    //Fractional note values

    //doted quarter note
    static constexpr float NDQ = 1.5f;
    //Doted Eighth
    static constexpr float NDE = 0.75f;
    //Eighth Note
    static constexpr float NE = 0.5f;
     //Sixteenth Note
    static constexpr float NS = 0.25f;
    //Thirty-second Note
    static constexpr float NT = 0.125f;

}//namespace music
#endif