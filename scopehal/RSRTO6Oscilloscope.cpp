/***********************************************************************************************************************
*                                                                                                                      *
* libscopehal v0.1                                                                                                     *
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

/*
 * Current State
 * =============
 * - Digital channels not implemented
 * - Only basic edge trigger supported. Coupling, hysteresis, B trigger not implemented
 *
 * RS Oscilloscope driver parts (c) 2021 Francisco Sedano, tested on RTM3004
 */


#include "scopehal.h"
#include "RSRTO6Oscilloscope.h"
#include "EdgeTrigger.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

RSRTO6Oscilloscope::RSRTO6Oscilloscope(SCPITransport* transport)
	: SCPIDevice(transport)
	, SCPIInstrument(transport)
	, m_triggerArmed(false)
	, m_triggerOneShot(false)
{
	LogDebug("m_model: %s\n", m_model.c_str());
	if (m_model != "RTO6")
	{
		LogFatal("rs.rto6 driver only appropriate for RTO6\n");
	}

	int nchans = 4;
	for(int i=0; i<nchans; i++)
	{
		//Hardware name of the channel
		string chname = string("CHAN1");
		chname[4] += i;

		//Color the channels based on R&S's standard color sequence (yellow-green-orange-bluegray)
		string color = "#ffffff";
		switch(i)
		{
			case 0:
				color = "#ffff00";
				break;

			case 1:
				color = "#00ff00";
				break;

			case 2:
				color = "#ff8000";
				break;

			case 3:
				color = "#8080ff";
				break;
		}

		//Create the channel
		auto chan = new OscilloscopeChannel(
			this,
			chname,
			color,
			Unit(Unit::UNIT_FS),
			Unit(Unit::UNIT_VOLTS),
			Stream::STREAM_TYPE_ANALOG,
			i);
		m_channels.push_back(chan);
		chan->SetDefaultDisplayName();

		// //Request all points when we download
		// m_transport->SendCommandQueued(chname + ":DATA:POIN MAX");
	}
	m_analogChannelCount = nchans;

	m_transport->SendCommandQueued("FORMat:DATA REAL,32");
	m_transport->SendCommandQueued("ACQuire:COUNt 1");
	m_transport->SendCommandQueued("EXPort:WAVeform:INCXvalues OFF");
	m_transport->SendCommandQueued("*WAI");
}

RSRTO6Oscilloscope::~RSRTO6Oscilloscope()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Accessors

string RSRTO6Oscilloscope::GetDriverNameInternal()
{
	return "rs.rto6";
}

unsigned int RSRTO6Oscilloscope::GetInstrumentTypes()
{
	return Instrument::INST_OSCILLOSCOPE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Device interface functions

void RSRTO6Oscilloscope::FlushConfigCache()
{
	lock_guard<recursive_mutex> lock(m_cacheMutex);

	m_channelOffsets.clear();
	m_channelVoltageRanges.clear();
	m_channelsEnabled.clear();
	m_channelCouplings.clear();
	m_channelAttenuations.clear();

	delete m_trigger;
	m_trigger = NULL;
}

bool RSRTO6Oscilloscope::IsChannelEnabled(size_t i)
{
	return i == 0;
}

void RSRTO6Oscilloscope::EnableChannel(size_t i)
{
	
}

void RSRTO6Oscilloscope::DisableChannel(size_t i)
{

}

vector<OscilloscopeChannel::CouplingType> RSRTO6Oscilloscope::GetAvailableCouplings(size_t /*i*/)
{
	vector<OscilloscopeChannel::CouplingType> ret;
	ret.push_back(OscilloscopeChannel::COUPLE_DC_1M);
	ret.push_back(OscilloscopeChannel::COUPLE_AC_1M);
	ret.push_back(OscilloscopeChannel::COUPLE_DC_50);
	return ret;
}

OscilloscopeChannel::CouplingType RSRTO6Oscilloscope::GetChannelCoupling(size_t i)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelCouplings.find(i) != m_channelCouplings.end())
			return m_channelCouplings[i];
	}

	string reply;
	{
		// lock_guard<recursive_mutex> lock(m_mutex);

		reply = m_transport->SendCommandQueuedWithReply(m_channels[i]->GetHwname() + ":COUP?");
	}
	OscilloscopeChannel::CouplingType coupling;

	if(reply == "AC")
		coupling = OscilloscopeChannel::COUPLE_AC_1M;
	else if(reply == "DCL" || reply == "DCLimit")
		coupling = OscilloscopeChannel::COUPLE_DC_1M;
	else if(reply == "DC")
		coupling = OscilloscopeChannel::COUPLE_DC_50;
	else
	{
		LogWarning("invalid coupling value\n");
		coupling = OscilloscopeChannel::COUPLE_DC_50;
	}

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelCouplings[i] = coupling;
	return coupling;
}

void RSRTO6Oscilloscope::SetChannelCoupling(size_t i, OscilloscopeChannel::CouplingType type)
{
	{
		// lock_guard<recursive_mutex> lock(m_mutex);
		switch(type)
		{
			case OscilloscopeChannel::COUPLE_DC_50:
				m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":COUP DC");
				break;

			case OscilloscopeChannel::COUPLE_AC_1M:
				m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":COUP AC");
				break;

			case OscilloscopeChannel::COUPLE_DC_1M:
				m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":COUP DCLimit");
				break;

			default:
				LogError("Invalid coupling for channel\n");
		}
	}
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelCouplings[i] = type;
}

double RSRTO6Oscilloscope::GetChannelAttenuation(size_t i)
{
	// {
	// 	lock_guard<recursive_mutex> lock(m_cacheMutex);
	// 	if(m_channelAttenuations.find(i) != m_channelAttenuations.end())
	// 		return m_channelAttenuations[i];
	// }
	// // FIXME Don't know SCPI to get this, relying on cache
	return 1;
}

void RSRTO6Oscilloscope::SetChannelAttenuation(size_t i, double atten)
{
	// {
	// 	lock_guard<recursive_mutex> lock(m_cacheMutex);
	// 	m_channelAttenuations[i] = atten;
	// }

	// lock_guard<recursive_mutex> lock(m_mutex);

	// char cmd[128];
	// snprintf(cmd, sizeof(cmd), "PROB%zd:SET:ATT:MAN ", m_channels[i]->GetIndex()+1);
	// PushFloat(cmd, atten);
}

unsigned int RSRTO6Oscilloscope::GetChannelBandwidthLimit(size_t /*i*/)
{
	return 0;
}

void RSRTO6Oscilloscope::SetChannelBandwidthLimit(size_t /*i*/, unsigned int /*limit_mhz*/)
{
	LogWarning("RSRTO6Oscilloscope::SetChannelBandwidthLimit unimplemented\n");
}

float RSRTO6Oscilloscope::GetChannelVoltageRange(size_t i, size_t /*stream*/)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelVoltageRanges.find(i) != m_channelVoltageRanges.end())
			return m_channelVoltageRanges[i];
	}

	// lock_guard<recursive_mutex> lock2(m_mutex);

	string reply = m_transport->SendCommandQueuedWithReply(m_channels[i]->GetHwname() + ":RANGE?");

	float range;
	sscanf(reply.c_str(), "%f", &range);
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelVoltageRanges[i] = range;
	return range;
}

void RSRTO6Oscilloscope::SetChannelVoltageRange(size_t i, size_t /*stream*/, float range)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_channelVoltageRanges[i] = range;
	}

	// lock_guard<recursive_mutex> lock(m_mutex);
	char cmd[128];
	snprintf(cmd, sizeof(cmd), "%s:RANGE %.4f", m_channels[i]->GetHwname().c_str(), range);
	m_transport->SendCommandQueued(cmd);
}

OscilloscopeChannel* RSRTO6Oscilloscope::GetExternalTrigger()
{
	//FIXME
	LogWarning("RSRTO6Oscilloscope::GetExternalTrigger unimplemented\n");
	return NULL;
}

float RSRTO6Oscilloscope::GetChannelOffset(size_t i, size_t /*stream*/)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);

		if(m_channelOffsets.find(i) != m_channelOffsets.end())
			return m_channelOffsets[i];
	}

	// lock_guard<recursive_mutex> lock2(m_mutex);

	string reply = m_transport->SendCommandQueuedWithReply(m_channels[i]->GetHwname() + ":OFFS?");

	float offset;
	sscanf(reply.c_str(), "%f", &offset);
	offset = -offset;
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelOffsets[i] = offset;
	return offset;
}

void RSRTO6Oscilloscope::SetChannelOffset(size_t i, size_t /*stream*/, float offset)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_channelOffsets[i] = offset;
	}

	// lock_guard<recursive_mutex> lock(m_mutex);
	char cmd[128];
	snprintf(cmd, sizeof(cmd), "%s:OFFS %.4f", m_channels[i]->GetHwname().c_str(), -offset);
	m_transport->SendCommandQueued(cmd);
}

Oscilloscope::TriggerMode RSRTO6Oscilloscope::PollTrigger()
{
	// lock_guard<recursive_mutex> lock(m_mutex);
	if (!m_triggerArmed)
		return TRIGGER_MODE_STOP;

	////////////////////////////////////////////////////////////////////////////
	string state = m_transport->SendCommandQueuedWithReply("ACQuire:CURRent?");

	if (state == "0")
	{
		return TRIGGER_MODE_RUN;
	}
	else if (state == "1")
	{
		m_triggerArmed = false;
		return TRIGGER_MODE_TRIGGERED;
	}
	else
	{
		LogWarning("ACQuire:CURRent? -> %s\n", state.c_str());
		return TRIGGER_MODE_TRIGGERED;
	}
}

bool RSRTO6Oscilloscope::AcquireData()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	lock_guard<recursive_mutex> lock2(m_transport->GetMutex());
	LogDebug(" ** AcquireData ** \n");

	auto start_time = std::chrono::system_clock::now();

	// m_transport->SendCommandQueued("*DCL; *WAI");

	LogIndenter li;

	map<int, vector<UniformAnalogWaveform*> > pending_waveforms;
	bool any_data = false;

	for(size_t i=0; i<m_analogChannelCount; i++)
	{
		if(!IsChannelEnabled(i))
			continue;

		//This is basically the same function as a LeCroy WAVEDESC, but much less detailed
		string reply = m_transport->SendCommandImmediateWithReply(m_channels[i]->GetHwname() + ":DATA:HEAD?; *WAI");

		double xstart;
		double xstop;
		size_t length;
		int samples_per_interval;
		int rc = sscanf(reply.c_str(), "%lf,%lf,%zu,%d", &xstart, &xstop, &length, &samples_per_interval);
		if (samples_per_interval != 1)
			LogFatal("Don't understand samples_per_interval != 1");

		if (rc != 4 || length == 0)
		{
			/* No data - Skip query the scope and move on */
			pending_waveforms[i].push_back(NULL);
			continue;
		}
		any_data = true;

		//Figure out the sample rate
		double capture_len_sec = xstop - xstart;
		double sec_per_sample = capture_len_sec / length;
		int64_t fs_per_sample = round(sec_per_sample * FS_PER_SECOND);
		LogDebug("%ld fs/sample\n", fs_per_sample);

		float* temp_buf = new float[length];

		//Set up the capture we're going to store our data into (no high res timer on R&S scopes)
		auto cap = new UniformAnalogWaveform;
		cap->m_timescale = fs_per_sample;
		cap->m_triggerPhase = 0;
		cap->m_startTimestamp = time(NULL);
		double t = GetTime();
		cap->m_startFemtoseconds = (t - floor(t)) * FS_PER_SECOND;

		//Ask for the data
		size_t len_bytes;
		float* samples = (float*)m_transport->SendCommandImmediateWithRawBlockReply(m_channels[i]->GetHwname() + ":DATA?; *WAI", len_bytes);

		if (len_bytes != (length*sizeof(float)))
		{
			LogFatal("Unexpected number of bytes back");
		}

		//Read the actual data.
		//Super easy, it comes across the wire in IEEE754 already!
		cap->Resize(length);
		cap->PrepareForCpuAccess();
		memcpy((unsigned char*)cap->m_samples.GetCpuPointer(), samples, len_bytes);
		cap->MarkSamplesModifiedFromCpu();

		delete[] samples;

		//Discard trailing newline
		uint8_t disregard;
		m_transport->ReadRawData(1, &disregard);

		//Done, update the data
		pending_waveforms[i].push_back(cap);

		//Clean up
		delete[] temp_buf;
	}

	if (any_data)
	{
		//Now that we have all of the pending waveforms, save them in sets across all channels
		m_pendingWaveformsMutex.lock();
		size_t num_pending = 1;	//TODO: segmented capture support
		for(size_t i=0; i<num_pending; i++)
		{
			SequenceSet s;
			for(size_t j=0; j<m_analogChannelCount; j++)
			{
				if(IsChannelEnabled(j))
					s[m_channels[j]] = pending_waveforms[j][i];
			}
			m_pendingWaveforms.push_back(s);
		}
		m_pendingWaveformsMutex.unlock();
	}

	if(!any_data || !m_triggerOneShot)
	{
		m_transport->SendCommandImmediate("SINGle");
		usleep(100000);
		// If we don't wait here, sending the query for available waveforms will race and return 1 for the exitisting waveform and jam everything up.
		m_triggerArmed = true;
	}

	auto end_time = std::chrono::system_clock::now();

	LogDebug("Acquisition took %lu\n", std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count());

	return any_data;
}

void RSRTO6Oscilloscope::Start()
{
	// lock_guard<recursive_mutex> lock(m_mutex);
	LogDebug("Start");
	m_transport->SendCommandImmediate("SINGle");
	usleep(100000);
	// If we don't wait here, sending the query for available waveforms will race and return 1 for the exitisting waveform and jam everything up.
	m_triggerArmed = true;
	m_triggerOneShot = false;
}

void RSRTO6Oscilloscope::StartSingleTrigger()
{
	// lock_guard<recursive_mutex> lock(m_mutex);
	LogDebug("Start oneshot");
	m_transport->SendCommandImmediate("SINGle");
	usleep(100000);
	// If we don't wait here, sending the query for available waveforms will race and return 1 for the exitisting waveform and jam everything up.
	m_triggerArmed = true;
	m_triggerOneShot = true;
}

void RSRTO6Oscilloscope::Stop()
{
	// lock_guard<recursive_mutex> lock(m_mutex);
	LogDebug("Stop!");
	m_transport->SendCommandImmediate("STOP");
	m_triggerArmed = false;
	m_triggerOneShot = true;
}

void RSRTO6Oscilloscope::ForceTrigger()
{
	LogError("RSRTO6Oscilloscope::ForceTrigger not implemented\n");
}

bool RSRTO6Oscilloscope::IsTriggerArmed()
{
	return m_triggerArmed;
}

vector<uint64_t> RSRTO6Oscilloscope::GetSampleRatesNonInterleaved()
{
	LogWarning("RSRTO6Oscilloscope::GetSampleRatesNonInterleaved unimplemented\n");

	//FIXME
	vector<uint64_t> ret;
	return ret;
}

vector<uint64_t> RSRTO6Oscilloscope::GetSampleRatesInterleaved()
{
	LogWarning("RSRTO6Oscilloscope::GetSampleRatesInterleaved unimplemented\n");

	//FIXME
	vector<uint64_t> ret;
	return ret;
}

set<Oscilloscope::InterleaveConflict> RSRTO6Oscilloscope::GetInterleaveConflicts()
{
	LogWarning("RSRTO6Oscilloscope::GetInterleaveConflicts unimplemented\n");

	//FIXME
	set<Oscilloscope::InterleaveConflict> ret;
	return ret;
}

vector<uint64_t> RSRTO6Oscilloscope::GetSampleDepthsNonInterleaved()
{
	LogWarning("RSRTO6Oscilloscope::GetSampleDepthsNonInterleaved unimplemented\n");

	//FIXME
	vector<uint64_t> ret;
	return ret;
}

vector<uint64_t> RSRTO6Oscilloscope::GetSampleDepthsInterleaved()
{
	LogWarning("RSRTO6Oscilloscope::GetSampleDepthsInterleaved unimplemented\n");

	//FIXME
	vector<uint64_t> ret;
	return ret;
}

uint64_t RSRTO6Oscilloscope::GetSampleRate()
{
	//FIXME
	return 1;
}

uint64_t RSRTO6Oscilloscope::GetSampleDepth()
{
	//FIXME
	return 1;
}

void RSRTO6Oscilloscope::SetSampleDepth(uint64_t /*depth*/)
{
	//FIXME
}

void RSRTO6Oscilloscope::SetSampleRate(uint64_t /*rate*/)
{
	//FIXME
}

void RSRTO6Oscilloscope::SetTriggerOffset(int64_t /*offset*/)
{
	//FIXME
}

int64_t RSRTO6Oscilloscope::GetTriggerOffset()
{
	//FIXME
	return 0;
}

bool RSRTO6Oscilloscope::IsInterleaving()
{
	return false;
}

bool RSRTO6Oscilloscope::SetInterleaving(bool /*combine*/)
{
	return false;
}

void RSRTO6Oscilloscope::PullTrigger()
{
	m_trigger = NULL;
}

/**
	@brief Reads settings for an edge trigger from the instrument
 */
void RSRTO6Oscilloscope::PullEdgeTrigger()
{
	
}

void RSRTO6Oscilloscope::PushTrigger()
{
	
}

/**
	@brief Pushes settings for an edge trigger to the instrument
 */
void RSRTO6Oscilloscope::PushEdgeTrigger(EdgeTrigger* trig)
{
	
}
