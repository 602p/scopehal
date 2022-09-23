/***********************************************************************************************************************
*                                                                                                                      *
* libscopeprotocols                                                                                                    *
*                                                                                                                      *
* Copyright (c) 2012-2022 Andrew D. Zonenberg and contributors                                                         *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

#include "../scopehal/scopehal.h"
#include "WaveformSubsetSelectionFilter.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

WaveformSubsetSelectionFilter::WaveformSubsetSelectionFilter(const string& color)
	: Filter(color, CAT_MATH)
	, m_startTimeName("Start Time")
	, m_durationName("Duration")
{
	AddStream(Unit(Unit::UNIT_VOLTS), "data", Stream::STREAM_TYPE_ANALOG);
	CreateInput("din");

	m_parameters[m_startTimeName] = FilterParameter(FilterParameter::TYPE_INT, Unit(Unit::UNIT_FS));
	m_parameters[m_startTimeName].SetIntVal(0);

	m_parameters[m_durationName] = FilterParameter(FilterParameter::TYPE_INT, Unit(Unit::UNIT_FS));
	m_parameters[m_durationName].SetFloatVal(FS_PER_SECOND / 10);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Factory methods

bool WaveformSubsetSelectionFilter::ValidateChannel(size_t i, StreamDescriptor stream)
{
	if(stream.m_channel == NULL)
		return false;

	if( (i == 0) && (stream.GetXAxisUnits() == Unit(Unit::UNIT_FS)) )
		return true;

	return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Accessors

string WaveformSubsetSelectionFilter::GetProtocolName()
{
	return "Select Waveform Subset";
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Actual decoder logic

void WaveformSubsetSelectionFilter::Refresh()
{
	//Make sure we've got valid inputs
	if(!VerifyAllInputsOK())
	{
		SetData(NULL, 0);
		return;
	}

	auto in = GetInputWaveform(0);

	int64_t start_time = m_parameters[m_startTimeName].GetIntVal();
	int64_t end_time = start_time + m_parameters[m_durationName].GetIntVal();

	int start_sample = GetIndexNearestAfterTimestamp(in, start_time);
	int end_sample = GetIndexNearestAfterTimestamp(in, end_time);

	if (start_sample == -1)
		start_sample = 0;

	if (end_sample == -1)
		end_sample = in->size() - 1;

	
	WaveformBase* cap;

	if(auto uaw = dynamic_cast<UniformAnalogWaveform*>(in))
	{
		// LogDebug("Case: uaw\n");
		m_streams[0].m_stype = Stream::STREAM_TYPE_ANALOG; // TODO: I think this races with WaveformArea::MapAllBuffers
		cap = SetupEmptyUniformAnalogOutputWaveform(uaw, 0);
		cap->Resize(end_sample - start_sample);
	}
	else if (auto saw = dynamic_cast<SparseAnalogWaveform*>(in))
	{
		// LogDebug("Case: saw\n");
		m_streams[0].m_stype = Stream::STREAM_TYPE_ANALOG; // TODO: I think this races with WaveformArea::MapAllBuffers
		cap = SetupSparseOutputWaveform(saw, 0, start_sample, saw->size() - end_sample);
	}
	else if(auto udw = dynamic_cast<UniformDigitalWaveform*>(in))
	{
		// LogDebug("Case: udw\n");
		m_streams[0].m_stype = Stream::STREAM_TYPE_DIGITAL; // TODO: I think this races with WaveformArea::MapAllBuffers
		cap = SetupEmptyUniformDigitalOutputWaveform(udw, 0);
		cap->Resize(end_sample - start_sample);
	}
	else if (auto sdw = dynamic_cast<SparseDigitalWaveform*>(in))
	{
		// LogDebug("Case: sdw\n");
		m_streams[0].m_stype = Stream::STREAM_TYPE_DIGITAL; // TODO: I think this races with WaveformArea::MapAllBuffers
		cap = SetupSparseDigitalOutputWaveform(sdw, 0, start_sample, sdw->size() - end_sample);
	}
	else
	{
		LogError("Unknown waveform type in WaveformSubsetSelectionFilter");
		return;
	}

	cap->PrepareForCpuAccess();

	uint8_t* out = GetAlignedSamplesPointer(cap);
	uint8_t* a = GetAlignedSamplesPointer(in);

	// LogDebug("Doing memcpy. start_sample=%d, end_sample=%d, GetSampleSize()=%ld, out=%p, a=%p\n", start_sample, end_sample, in->GetSampleSize(), out, a);

	memcpy(&out[0], &a[start_sample * in->GetSampleSize()], (end_sample - start_sample) * in->GetSampleSize());

	cap->m_triggerPhase = GetOffsetScaled(in, start_sample);

	cap->MarkModifiedFromCpu();
}
