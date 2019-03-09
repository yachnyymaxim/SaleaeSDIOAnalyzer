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

#include "SDIOAnalyzer.h"
#include "SDIOAnalyzerSettings.h"
#include <AnalyzerChannelData.h>

SDIOAnalyzer::SDIOAnalyzer()
:    Analyzer2(),
    mSettings( new SDIOAnalyzerSettings() ),
    mSimulationInitilized( false ),
    mAlreadyRun(false),
    packetState(WAITING_FOR_PACKET),
    frameState(TRANSMISSION_BIT)
{
    SetAnalyzerSettings( mSettings.get() );
}

SDIOAnalyzer::~SDIOAnalyzer()
{
    KillThread();
}

void SDIOAnalyzer::SetupResults()
{
	mResults.reset( new SDIOAnalyzerResults( this, mSettings.get() ) );
	SetAnalyzerResults(mResults.get());
}

void SDIOAnalyzer::WorkerThread()
{
    mAlreadyRun = true;

    // mResults->AddChannelBubblesWillAppearOn(mSettings->mClockChannel);
    mResults->AddChannelBubblesWillAppearOn(mSettings->mCmdChannel);
    // mResults->AddChannelBubblesWillAppearOn(mSettings->mDAT0Channel);
    // mResults->AddChannelBubblesWillAppearOn(mSettings->mDAT1Channel);
    // mResults->AddChannelBubblesWillAppearOn(mSettings->mDAT2Channel);
    // mResults->AddChannelBubblesWillAppearOn(mSettings->mDAT3Channel);

    mClock = GetAnalyzerChannelData(mSettings->mClockChannel);
    mCmd = GetAnalyzerChannelData(mSettings->mCmdChannel);
    mDAT0 = GetAnalyzerChannelData(mSettings->mDAT0Channel);
    mDAT1 = mSettings->mDAT1Channel == UNDEFINED_CHANNEL ? nullptr : GetAnalyzerChannelData(mSettings->mDAT1Channel);
    mDAT2 = mSettings->mDAT2Channel == UNDEFINED_CHANNEL ? nullptr : GetAnalyzerChannelData(mSettings->mDAT2Channel);
    mDAT3 = mSettings->mDAT3Channel == UNDEFINED_CHANNEL ? nullptr : GetAnalyzerChannelData(mSettings->mDAT3Channel);

    mClock->AdvanceToNextEdge();
    mCmd->AdvanceToAbsPosition(mClock->GetSampleNumber());
    mDAT0->AdvanceToAbsPosition(mClock->GetSampleNumber());
    if (mDAT1) mDAT1->AdvanceToAbsPosition(mClock->GetSampleNumber());
    if (mDAT2) mDAT2->AdvanceToAbsPosition(mClock->GetSampleNumber());
    if (mDAT3) mDAT3->AdvanceToAbsPosition(mClock->GetSampleNumber());

    for ( ; ; ){
        PacketStateMachine();

        mResults->CommitResults();
        ReportProgress(mClock->GetSampleNumber());
    }
}

//Determine whether or not we are in a packet
void SDIOAnalyzer::PacketStateMachine()
{
    if (packetState == WAITING_FOR_PACKET)
    {
        //If we are not in a packet, let's advance to the next edge on the
        //command line
        mCmd->AdvanceToNextEdge();
        U64 sampleNumber = mCmd->GetSampleNumber();
        lastFallingClockEdge = sampleNumber;
        mClock->AdvanceToAbsPosition(sampleNumber);
        //After advancing to the next command line edge the clock can either
        //high or low.  If it is high, we need to advance two clock edges.  If
        //it is low, we only need to advance one clock edge.
        if (mClock->GetBitState() == BIT_HIGH){
            mClock->AdvanceToNextEdge();
        }

        mClock->AdvanceToNextEdge();
        sampleNumber = mClock->GetSampleNumber();

        mCmd->AdvanceToAbsPosition(sampleNumber);
        mDAT0->AdvanceToAbsPosition(sampleNumber);
        if (mDAT1) mDAT1->AdvanceToAbsPosition(sampleNumber);
        if (mDAT2) mDAT2->AdvanceToAbsPosition(sampleNumber);
        if (mDAT3) mDAT3->AdvanceToAbsPosition(sampleNumber);

        if (mCmd->GetBitState() == BIT_LOW){
            packetState = IN_PACKET;
        }


    }
    else if (packetState == IN_PACKET)
    {
        mClock->AdvanceToNextEdge();
        U64 sampleNumber = mClock->GetSampleNumber();

        mCmd->AdvanceToAbsPosition(sampleNumber);
        mDAT0->AdvanceToAbsPosition(sampleNumber);
        if (mDAT1) mDAT1->AdvanceToAbsPosition(sampleNumber);
        if (mDAT2) mDAT2->AdvanceToAbsPosition(sampleNumber);
        if (mDAT3) mDAT3->AdvanceToAbsPosition(sampleNumber);

        if (mClock->GetBitState() == BIT_HIGH){
            mResults->AddMarker(mClock->GetSampleNumber(),
                AnalyzerResults::UpArrow, mSettings->mClockChannel);
            if (FrameStateMachine()==1){
                mResults->CommitPacketAndStartNewPacket();
                mResults->CommitResults();
                packetState = WAITING_FOR_PACKET;
            }
        }else{
            lastFallingClockEdge = mClock->GetSampleNumber();
        }
    }
}


//This state machine will deal with accepting the different parts of the
//transmitted information.  In order to correctly interpret the data stream,
//we need to be able to distinguish between 4 different kinds of packets.
//They are:
//    - Command
//      - Short Response
//  - Long Response
//  - Data

U32 SDIOAnalyzer::FrameStateMachine()
{
    if (frameState == TRANSMISSION_BIT)
    {
        Frame frame;
        frame.mStartingSampleInclusive = lastFallingClockEdge;
        frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
        frame.mFlags = 0;
        frame.mData1 = mCmd->GetBitState();
        frame.mType = FRAME_DIR;
        mResults->AddFrame(frame);

        //The transmission bit tells us the origin of the packet
        //If the bit is high the packet comes from the host
        //If the bit is low, the packet comes from the slave
        isCmd = mCmd->GetBitState();


        frameState = COMMAND;
        frameCounter = 6;

        startOfNextFrame = (frame.mEndingSampleInclusive + 1);
        temp = 0;
    }
    else if (frameState == COMMAND)
    {
        temp = temp<<1 | mCmd->GetBitState();

        frameCounter--;
        if (frameCounter == 0)
        {
            Frame frame;
            frame.mStartingSampleInclusive = startOfNextFrame;
            frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
            frame.mFlags = 0;
            frame.mData1 = temp; // Select the first 6 bits
            frame.mType = FRAME_CMD;
            mResults->AddFrame(frame);

            //Once we have the arguement

            //Find the expected length of the next reponse based on the command
            if (isCmd && app){
                //Deal with the application commands first
                //All Application commands have a 48 bit response
                respLength = 32;
            }else if (isCmd){
                //Deal with standard commands now
                //CMD2, CMD9 and CMD10 respond with R2
                if (temp == 2 || temp == 9 || temp == 10){
                    respLength = 127;
                    respType = RESP_LONG;
                }else{
                    // All others have 48 bit responses
                    respLength = 32;
                    respType = RESP_NORMAL;
                }

            }

            if (temp == 52)
              {
                frameState = CMD52_ARGUMENT;

                cmd52State = isCmd ? CMD52_RWFLAG : CMD52_RESP_STUFF;
              }
            else if (temp == 53)
              {
                frameState = CMD53_ARGUMENT;
                // Are we decoding a command from host or response from device
                // Based on this choose which state to start in
                cmd53State = isCmd ? CMD53_RWFLAG : CMD53_RESP_STUFF;
              }
            else
              {
                frameState = ARGUMENT;
              }

            startOfNextFrame = frame.mEndingSampleInclusive + 1;
            temp = 0;
            if (isCmd){
                frameCounter = 32;
            }else{
                frameCounter = respLength;
            }
        }

    }
    else if (frameState == ARGUMENT)
    {
        temp = temp << 1 | mCmd->GetBitState();

        frameCounter--;

        if (!isCmd && frameCounter == 1 && respType == RESP_LONG){
            temp = temp<<1;

            Frame frame;
            frame.mStartingSampleInclusive = startOfNextFrame;
            frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
            frame.mFlags = 0;
            frame.mData1 = temp2;
            frame.mData2 = temp;
            frame.mType = FRAME_LONG_ARG;
            mResults->AddFrame(frame);

            frameState = STOP;
            frameCounter = 1;
            startOfNextFrame = frame.mEndingSampleInclusive + 1;
            temp = 0;

        }else if (frameCounter == 0){
            Frame frame;
            frame.mStartingSampleInclusive = startOfNextFrame;
            frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
            frame.mFlags = 0;
            frame.mData1 = temp; // Select the first 6 bits
            frame.mType = FRAME_ARG;
            mResults->AddFrame(frame);

            frameState = CRC7;
            frameCounter = 7;
            startOfNextFrame = frame.mEndingSampleInclusive + 1;
            temp = 0;
        }else if (frameCounter == 63 && !isCmd){
            temp2 = temp;
            temp = 0;
        }
    }
    else if (frameState == CMD52_ARGUMENT)
    {
        temp = temp << 1 | mCmd->GetBitState();

        frameCounter--;
        if (cmd52State == CMD52_RWFLAG)
          {
            Frame frame;
            frame.mStartingSampleInclusive = startOfNextFrame;
            frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
            frame.mFlags = 0;
            frame.mData1 = mCmd->GetBitState();
            frame.mType = FRAME_CMD52_RWFLAG;
            mResults->AddFrame(frame);

            cmd52State = CMD52_FN;
            startOfNextFrame = frame.mEndingSampleInclusive + 1;
            // Keep track of if we're reading or writing
            cmd52writenotread = mCmd->GetBitState() ? true : false;
            temp = 0;
          }
        else if (cmd52State == CMD52_FN && frameCounter == 28)
          {
            Frame frame;
            frame.mStartingSampleInclusive = startOfNextFrame;
            frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
            frame.mFlags = 0;
            frame.mData1 = temp;
            frame.mType = FRAME_CMD52_FN;
            mResults->AddFrame(frame);

            cmd52State = CMD52_RAW;
            startOfNextFrame = frame.mEndingSampleInclusive + 1;
          }
        else if (cmd52State == CMD52_RAW)
          {
            Frame frame;
            frame.mStartingSampleInclusive = startOfNextFrame;
            frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
            frame.mFlags = 0;
            frame.mData1 = mCmd->GetBitState();
            frame.mType = FRAME_CMD52_RAW;
            mResults->AddFrame(frame);

            cmd52State = CMD52_STUFF1;
            startOfNextFrame = frame.mEndingSampleInclusive + 1;
            temp = 0;
          }
        else if (cmd52State == CMD52_STUFF1)
          {
            Frame frame;
            frame.mStartingSampleInclusive = startOfNextFrame;
            frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
            frame.mFlags = 0;
            frame.mData1 = mCmd->GetBitState();
            frame.mData2 = 1;
            frame.mType = FRAME_CMD52_STUFF;
            mResults->AddFrame(frame);

            cmd52State = CMD52_ADDR;
            startOfNextFrame = frame.mEndingSampleInclusive + 1;
            temp = 0;
          }
        else if (cmd52State == CMD52_ADDR && frameCounter == 9)
          {
            Frame frame;
            frame.mStartingSampleInclusive = startOfNextFrame;
            frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
            frame.mFlags = 0;
            frame.mData1 = temp;
            frame.mType = FRAME_CMD52_ADDR;
            mResults->AddFrame(frame);

            cmd52State = CMD52_STUFF2;
            startOfNextFrame = frame.mEndingSampleInclusive + 1;
            temp = 0;
          }
        else if (cmd52State == CMD52_STUFF2 && ((cmd52writenotread == false && frameCounter == 0)
                            || cmd52writenotread == true))
          {
            // If we're writing, we'll only have a single STUFF2 bit,
            // else if we're reading it'll be stuff bits until the end of the CMD52 argument field.
            Frame frame;
            frame.mStartingSampleInclusive = startOfNextFrame;
            frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
            frame.mFlags = 0;
            frame.mData1 = temp;
            frame.mType = FRAME_CMD52_STUFF;

            if (cmd52writenotread)
              {
            cmd52State = CMD52_DATA;
            frame.mData2 = 1; // Store the length of the stuff bits in this case in data2
              }
            else
              {
            frame.mData2 = 9; // Store the length of the stuff bits in this case in data2
            frameState = CRC7;
            frameCounter = 7;
              }
            mResults->AddFrame(frame);

            startOfNextFrame = frame.mEndingSampleInclusive + 1;
            temp = 0;
          }
        else if (cmd52State == CMD52_DATA && frameCounter == 0)
          {
            Frame frame;
            frame.mStartingSampleInclusive = startOfNextFrame;
            frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
            frame.mFlags = 0;
            frame.mData1 = temp;
            frame.mType = FRAME_CMD52_DATA;
            mResults->AddFrame(frame);

            frameState = CRC7;
            frameCounter = 7;
            startOfNextFrame = frame.mEndingSampleInclusive + 1;
            temp = 0;
          }
        else if (cmd52State == CMD52_RESP_STUFF && frameCounter == 16)
          {
            Frame frame;
            frame.mStartingSampleInclusive = startOfNextFrame;
            frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
            frame.mFlags = 0;
            frame.mData1 = temp;
            frame.mData2 = 16;
            frame.mType = FRAME_CMD52_STUFF;
            mResults->AddFrame(frame);

            cmd52State = CMD52_RESP_FLAGS;
            startOfNextFrame = frame.mEndingSampleInclusive + 1;
            temp = 0;
          }
        else if (cmd52State == CMD52_RESP_FLAGS && frameCounter == 8)
          {
            Frame frame;
            frame.mStartingSampleInclusive = startOfNextFrame;
            frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
            frame.mFlags = 0;
            frame.mData1 = temp;
            frame.mData2 = 16;
            frame.mType = FRAME_CMD52_FLAGS;
            mResults->AddFrame(frame);

            cmd52State = CMD52_DATA;
            startOfNextFrame = frame.mEndingSampleInclusive + 1;
            temp = 0;
          }
    }
    else if (frameState == CMD53_ARGUMENT)
    {
        temp = temp << 1 | mCmd->GetBitState();

        frameCounter--;
        if (cmd53State == CMD53_RWFLAG)
          {
            Frame frame;
            frame.mStartingSampleInclusive = startOfNextFrame;
            frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
            frame.mFlags = 0;
            frame.mData1 = mCmd->GetBitState();
            frame.mType = FRAME_CMD52_RWFLAG;
            mResults->AddFrame(frame);

            cmd53State = CMD53_FN;
            startOfNextFrame = frame.mEndingSampleInclusive + 1;
            temp = 0;
          }
        else if (cmd53State == CMD53_FN && frameCounter == 28)
          {
            Frame frame;
            frame.mStartingSampleInclusive = startOfNextFrame;
            frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
            frame.mFlags = 0;
            frame.mData1 = temp;
            frame.mType = FRAME_CMD52_FN;
            mResults->AddFrame(frame);

            cmd53State = CMD53_BLOCK;
            startOfNextFrame = frame.mEndingSampleInclusive + 1;
          }
        else if (cmd53State == CMD53_BLOCK)
          {
            Frame frame;
            frame.mStartingSampleInclusive = startOfNextFrame;
            frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
            frame.mFlags = 0;
            frame.mData1 = mCmd->GetBitState();
            frame.mType = FRAME_CMD53_BLOCK;
            mResults->AddFrame(frame);

            cmd53State = CMD53_OP;
            startOfNextFrame = frame.mEndingSampleInclusive + 1;
            temp = 0;
          }
        else if (cmd53State == CMD53_OP)
          {
            Frame frame;
            frame.mStartingSampleInclusive = startOfNextFrame;
            frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
            frame.mFlags = 0;
            frame.mData1 = mCmd->GetBitState();
            frame.mType = FRAME_CMD53_OP;
            mResults->AddFrame(frame);

            cmd53State = CMD53_ADDR;
            startOfNextFrame = frame.mEndingSampleInclusive + 1;
            temp = 0;
          }
        else if (cmd53State == CMD53_ADDR && frameCounter == 9)
          {
            Frame frame;
            frame.mStartingSampleInclusive = startOfNextFrame;
            frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
            frame.mFlags = 0;
            frame.mData1 = temp;
            frame.mType = FRAME_CMD52_ADDR;
            mResults->AddFrame(frame);

            cmd53State = CMD53_COUNT;
            startOfNextFrame = frame.mEndingSampleInclusive + 1;
            temp = 0;
          }
        else if (cmd53State == CMD53_COUNT && frameCounter == 0)
          {
            Frame frame;
            frame.mStartingSampleInclusive = startOfNextFrame;
            frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
            frame.mFlags = 0;
            frame.mData1 = temp;
            frame.mType = FRAME_CMD53_COUNT;
            mResults->AddFrame(frame);

            frameState = CRC7;
            frameCounter = 7;
            startOfNextFrame = frame.mEndingSampleInclusive + 1;
            temp = 0;
          }
        else if (cmd53State == CMD53_RESP_STUFF && frameCounter == 16)
          {
            Frame frame;
            frame.mStartingSampleInclusive = startOfNextFrame;
            frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
            frame.mFlags = 0;
            frame.mData1 = temp;
            frame.mData2 = 16;
            frame.mType = FRAME_CMD52_STUFF;
            mResults->AddFrame(frame);

            cmd53State = CMD53_RESP_FLAGS;
            startOfNextFrame = frame.mEndingSampleInclusive + 1;
            temp = 0;
          }
        else if (cmd53State == CMD53_RESP_FLAGS && frameCounter == 8)
          {
            Frame frame;
            frame.mStartingSampleInclusive = startOfNextFrame;
            frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
            frame.mFlags = 0;
            frame.mData1 = temp;
            frame.mData2 = 16;
            frame.mType = FRAME_CMD52_FLAGS;
            mResults->AddFrame(frame);

            cmd53State = CMD53_RESP_STUFF2;
            startOfNextFrame = frame.mEndingSampleInclusive + 1;
            temp = 0;
          }
        else if (cmd53State == CMD53_RESP_STUFF2 && frameCounter == 0)
          {
            Frame frame;
            frame.mStartingSampleInclusive = startOfNextFrame;
            frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
            frame.mFlags = 0;
            frame.mData1 = temp;
            frame.mData2 = 16;
            frame.mType = FRAME_CMD52_STUFF;
            mResults->AddFrame(frame);

            frameState = CRC7;
            frameCounter = 7;
            startOfNextFrame = frame.mEndingSampleInclusive + 1;
            temp = 0;
          }
    }
    else if (frameState == CRC7)
    {
        temp = temp << 1 | mCmd->GetBitState();

        frameCounter--;
        if (frameCounter == 0){
            Frame frame;
            frame.mStartingSampleInclusive = startOfNextFrame;
            frame.mEndingSampleInclusive = mClock->GetSampleOfNextEdge() - 1;
            frame.mFlags = 0;
            frame.mData1 = temp; // Select the first 6 bits
            frame.mType = FRAME_CRC;
            mResults->AddFrame(frame);

            frameState = STOP;
            startOfNextFrame = frame.mEndingSampleInclusive + 1;
            temp = 0;
        }
    }
    else if (frameState == STOP)
    {
        frameState = TRANSMISSION_BIT;
        return 1;
    }
    return 0;
}

bool SDIOAnalyzer::NeedsRerun()
{
    return !mAlreadyRun;
}

U32 SDIOAnalyzer::GenerateSimulationData( U64 minimum_sample_index, U32 device_sample_rate, SimulationChannelDescriptor** simulation_channels )
{
    return 0;
}

U32 SDIOAnalyzer::GetMinimumSampleRateHz()
{
    return 25000;
}

const char* SDIOAnalyzer::GetAnalyzerName() const
{
    return "SDIO";
}

const char* GetAnalyzerName()
{
    return "SDIO";
}

Analyzer* CreateAnalyzer()
{
    return new SDIOAnalyzer();
}

void DestroyAnalyzer( Analyzer* analyzer )
{
    delete analyzer;
}
