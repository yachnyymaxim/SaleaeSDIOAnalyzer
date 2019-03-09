// The MIT License (MIT)
//
// Copyright (c) 2013 Erick Fuentes http://erickfuent.es
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "SDIOAnalyzerResults.h"
#include <AnalyzerHelpers.h>
#include "SDIOAnalyzer.h"
#include "SDIOAnalyzerSettings.h"
#include <iostream>
#include <fstream>
#include <sstream>

SDIOAnalyzerResults::SDIOAnalyzerResults( SDIOAnalyzer* analyzer, SDIOAnalyzerSettings* settings )
:    AnalyzerResults(),
    mSettings( settings ),
    mAnalyzer( analyzer ),
    mLastPacket(-1),
    mLastFrame(-1)
{
}

SDIOAnalyzerResults::~SDIOAnalyzerResults()
{
}

void SDIOAnalyzerResults::GenerateBubbleText( U64 frame_index, Channel& channel, DisplayBase display_base )
{
    ClearResultStrings();
    Frame frame = GetFrame( frame_index );

    char number_str1[128];
    char number_str2[128];
    if (frame.mType == SDIOAnalyzer::FRAME_DIR){
        if (frame.mData1){
            AddResultString("H");
            AddResultString("Host");
            AddResultString("DIR: Host");
        }else{
            AddResultString("S");
            AddResultString("Slave");
            AddResultString("DIR: Slave");
        }
    }else if (frame.mType == SDIOAnalyzer::FRAME_CMD){
        AnalyzerHelpers::GetNumberString( frame.mData1, Decimal, 6, number_str1, 128 );
        AddResultString("CMD ", number_str1);
    }else if (frame.mType == SDIOAnalyzer::FRAME_ARG){
        AnalyzerHelpers::GetNumberString( frame.mData1, display_base, 32, number_str1, 128 );
        AddResultString("ARG ", number_str1);
    }else if (frame.mType == SDIOAnalyzer::FRAME_LONG_ARG){
        AnalyzerHelpers::GetNumberString (frame.mData1, display_base, 64, number_str1, 128);
        AnalyzerHelpers::GetNumberString (frame.mData2, display_base, 64, number_str2, 128);
        AddResultString("LONG: ", number_str1, number_str2);

    }else if (frame.mType == SDIOAnalyzer::FRAME_CRC){
        AnalyzerHelpers::GetNumberString( frame.mData1, display_base, 7, number_str1, 128 );
        AddResultString("CRC ", number_str1);
    }else if (frame.mType == SDIOAnalyzer::FRAME_CMD52_RWFLAG){
      if (frame.mData1)
        {
          AddResultString("W");
          //AddResultString("Write");
          //AddResultString("Register Write");
        }
      else
        {
          AddResultString("R");
          //AddResultString("Read");
          //AddResultString("Register Read");
        }
    }else if (frame.mType == SDIOAnalyzer::FRAME_CMD52_FN){
        AnalyzerHelpers::GetNumberString( frame.mData1, Decimal, 3, number_str1, 128 );
        AddResultString("F", number_str1);
        AddResultString("Func: ", number_str1);
        //AddResultString("Function: ", number_str1);
    }else if (frame.mType == SDIOAnalyzer::FRAME_CMD52_RAW){
        AnalyzerHelpers::GetNumberString( frame.mData1, Decimal, 1, number_str1, 128 );
        AddResultString("RAW: ", number_str1);
        AddResultString("Read after write: ", number_str1);
    }else if (frame.mType == SDIOAnalyzer::FRAME_CMD52_STUFF){
        AnalyzerHelpers::GetNumberString( frame.mData1, display_base, frame.mData2, number_str1, 128 );
        AddResultString("D/C");
        AddResultString("Stuff bits: ", number_str1);
    }else if (frame.mType == SDIOAnalyzer::FRAME_CMD52_ADDR){
        AnalyzerHelpers::GetNumberString( frame.mData1, display_base, 17, number_str1, 128 );
        AddResultString("Addr: ", number_str1);
    }else if (frame.mType == SDIOAnalyzer::FRAME_CMD52_DATA){
        AnalyzerHelpers::GetNumberString( frame.mData1, display_base, 8, number_str1, 128 );
        AddResultString("Data: ", number_str1);
    }else if (frame.mType == SDIOAnalyzer::FRAME_CMD52_FLAGS){
        AnalyzerHelpers::GetNumberString( frame.mData1, Binary, 8, number_str1, 128 );
        AddResultString("Response flags: ", number_str1);
        AddResultString("F: ", number_str1);
    }else if (frame.mType == SDIOAnalyzer::FRAME_CMD53_BLOCK){
      AnalyzerHelpers::GetNumberString( frame.mData1, Decimal, 1, number_str1, 128 );
      AddResultString("B: ", number_str1);
      AddResultString("Block: ", number_str1);
      AddResultString("Block mode: ", number_str1);
    }else if (frame.mType == SDIOAnalyzer::FRAME_CMD53_OP){
      AnalyzerHelpers::GetNumberString( frame.mData1, Decimal, 1, number_str1, 128 );
      AddResultString("Op: ", number_str1);
    }else if (frame.mType == SDIOAnalyzer::FRAME_CMD53_COUNT){
      AnalyzerHelpers::GetNumberString( frame.mData1, display_base, 9, number_str1, 128 );
      AddResultString("C: ", number_str1);
      AddResultString("Count: ", number_str1);
    }
}

void SDIOAnalyzerResults::GenerateExportFile( const char* file, DisplayBase display_base, U32 export_type_user_id )
{
    std::ofstream file_stream( file, std::ios::out );

    U64 trigger_sample = mAnalyzer->GetTriggerSample();
    U32 sample_rate = mAnalyzer->GetSampleRate();

    file_stream << "Time [s],Value" << std::endl;

    U64 num_packets = GetNumPackets();
    file_stream << num_packets << std::endl;

    for( U64 i = 0; i < num_packets; i++ )
    {
        char time_str[128];
        U64 first_frame_id, last_frame_id;

        GetFramesContainedInPacket(i, &first_frame_id, &last_frame_id );

        Frame frame = GetFrame(first_frame_id);

        AnalyzerHelpers::GetTimeString(frame.mStartingSampleInclusive, trigger_sample, sample_rate, time_str, 128);

        file_stream << std::endl << time_str << ", ";

        GeneratePacketDescription(i, display_base, file_stream);

        if( UpdateExportProgressAndCheckForCancel( i, num_packets ) == true )
        {
            file_stream.close();
            return;
        }
    }

    UpdateExportProgressAndCheckForCancel( num_packets, num_packets );

    file_stream.close();
}

void SDIOAnalyzerResults::GenerateFrameTabularText( U64 frame_index, DisplayBase display_base )
{
    // Packet-based print since SDIOAnalyzerResults::GeneratePacketTabularText is not used by the SDK
    U64 packet = GetPacketContainingFrame(frame_index);

    // Looks like this function is called several times using the same frame_index - we need to re-generate the
    // tabular text, otherwise it is re-written by "else" clause.
    if (mLastPacket != packet || mLastFrame == frame_index) {
        mLastPacket = packet;
        mLastFrame = frame_index;

        std::stringstream stream;
        GeneratePacketDescription(packet, display_base, stream);
        ClearTabularText();
        AddTabularText(stream.str().c_str());
    } else {
        // Clearing the text at all so that is is not shown for frames we are not interested in
        ClearTabularText();
    }
}

void SDIOAnalyzerResults::GeneratePacketTabularText( U64 packet_id, DisplayBase display_base )
{
    std::stringstream stream;

    GeneratePacketDescription(packet_id, display_base, stream);
    ClearTabularText();
    AddTabularText(stream.str().c_str());
}

void SDIOAnalyzerResults::GenerateTransactionTabularText( U64 transaction_id, DisplayBase display_base )
{

}

void SDIOAnalyzerResults::GeneratePacketDescription(U64 packet_id, DisplayBase display_base, std::ostream &stream) {

    U64 first_frame_id, last_frame_id;
    GetFramesContainedInPacket(packet_id, &first_frame_id, &last_frame_id);

    for( U64 i = first_frame_id; i <= last_frame_id; i++ )
    {
        Frame frame = GetFrame( i );

        char number_str1[128];
        char number_str2[128];
        if (frame.mType == SDIOAnalyzer::FRAME_DIR)
        {
            if (frame.mData1)
            {
                stream << "H->S | ";
            }
            else
            {
                stream << "S->H | ";
            }
        }
        else if (frame.mType == SDIOAnalyzer::FRAME_CMD)
        {
            AnalyzerHelpers::GetNumberString(frame.mData1, Decimal, 6, number_str1, 128);
            stream << "CMD: " << number_str1 << " | ";
        }
        else if (frame.mType == SDIOAnalyzer::FRAME_ARG)
        {
            AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 32, number_str1, 128);
            stream << "ARG: " << number_str1 << " | ";
        }
        else if (frame.mType == SDIOAnalyzer::FRAME_LONG_ARG)
        {
            AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 64, number_str1, 128);
            AnalyzerHelpers::GetNumberString(frame.mData2, display_base, 64, number_str2, 128);
            stream << "LARG: " << number_str1 << " " << number_str2 << " | ";
        }
        else if (frame.mType == SDIOAnalyzer::FRAME_CRC)
        {
            AnalyzerHelpers::GetNumberString(frame.mData1, Hexadecimal, 7, number_str1, 128);
            stream << "CRC: " << number_str1 << " | ";
        }
        else if (frame.mType == SDIOAnalyzer::FRAME_CMD52_RWFLAG)
        {
            if (frame.mData1)
            {
                stream << "W |";
            }
            else
            {
                stream << "R |";
            }
        }
        else if (frame.mType == SDIOAnalyzer::FRAME_CMD52_FN)
        {
            AnalyzerHelpers::GetNumberString(frame.mData1, Decimal, 3, number_str1, 128);
            stream << "Func: " << number_str1 << " | ";
        }
        else if (frame.mType == SDIOAnalyzer::FRAME_CMD52_RAW)
        {
            AnalyzerHelpers::GetNumberString(frame.mData1, Decimal, 1, number_str1, 128);
            stream << "Read after write: " << number_str1 << " | ";
        }
        else if (frame.mType == SDIOAnalyzer::FRAME_CMD52_STUFF)
        {
            //AnalyzerHelpers::GetNumberString(frame.mData1, display_base, frame.mData2, number_str1, 128);
            //stream << "Stuff bits: " << number_str1 << " | ";
        }
        else if (frame.mType == SDIOAnalyzer::FRAME_CMD52_ADDR)
        {
            AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 17, number_str1, 128);
            stream << "Addr: " << number_str1 << " | ";
        }
        else if (frame.mType == SDIOAnalyzer::FRAME_CMD52_DATA)
        {
            AnalyzerHelpers::GetNumberString(frame.mData1, Hexadecimal, 8, number_str1, 128);
            stream << "Data: " << number_str1 << " | ";
        }
        else if (frame.mType == SDIOAnalyzer::FRAME_CMD52_FLAGS)
        {
            AnalyzerHelpers::GetNumberString(frame.mData1, Binary, 8, number_str1, 128);
            stream << "Response flags: " << number_str1 << " | ";
        }
        else if (frame.mType == SDIOAnalyzer::FRAME_CMD53_BLOCK)
        {
            AnalyzerHelpers::GetNumberString(frame.mData1, Decimal, 1, number_str1, 128);
            stream << "Block mode: " << number_str1 << " | ";
        }
        else if (frame.mType == SDIOAnalyzer::FRAME_CMD53_OP)
        {
            AnalyzerHelpers::GetNumberString(frame.mData1, Decimal, 1, number_str1, 128);
            stream << "Op: " << number_str1 << " | ";
        }
        else if (frame.mType == SDIOAnalyzer::FRAME_CMD53_COUNT)
        {
            AnalyzerHelpers::GetNumberString(frame.mData1, Decimal, 9, number_str1, 128);
            stream << "Count: " << number_str1 << " | ";
        }

    } // for( U64 i = first_frame_id; i <= last_frame_id; i++ )

}
