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
	, m_sampleRateValid(false)
	, m_sampleDepthValid(false)
{
	LogDebug("m_model: %s\n", m_model.c_str());
	if (m_model != "RTO6")
	{
		LogFatal("rs.rto6 driver only appropriate for RTO6");
	}

	SCPISocketTransport* sockettransport = NULL;

	if (!(sockettransport = dynamic_cast<SCPISocketTransport*>(transport)))
	{
		LogFatal("rs.rto6 driver requires 'lan' transport");
	}

	m_secondSocket = new SCPISocketTransport(sockettransport->GetHostname(), sockettransport->GetPort());
	string idn2 = m_secondSocket->SendCommandImmediateWithReply("*IDN?");
	LogDebug("idn2: %s\n", idn2.c_str());

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

		// TODO: Detect probe
	}
	m_analogChannelCount = nchans;

	m_transport->SendCommandQueued("FORMat:DATA REAL,32"); //Report in f32
	m_transport->SendCommandQueued("ACQuire:COUNt 1"); //Limit to one acquired waveform per "SINGLE"
	m_transport->SendCommandQueued("EXPort:WAVeform:INCXvalues OFF"); //Don't include X values in data
	m_transport->SendCommandQueued("TIMebase:ROLL:ENABle OFF"); //No roll mode
	m_transport->SendCommandQueued("TRIGGER1:MODE NORMAL"); //No auto trigger
	m_transport->SendCommandQueued("ACQuire:CDTA ON"); //All channels have same timebase/etc
	m_transport->SendCommandQueued("PROBE1:SETUP:ATT:MODE MAN"); //Allow/use manual attenuation setting with unknown probes
	m_transport->SendCommandQueued("*WAI");

	GetSampleDepth();
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
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);

		if(m_channelsEnabled.find(i) != m_channelsEnabled.end())
			return m_channelsEnabled[i];
	}

	string reply = m_transport->SendCommandQueuedWithReply(
						m_channels[i]->GetHwname() + ":STATE?");

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelsEnabled[i] = (reply == "1");
	return m_channelsEnabled[i];
}

void RSRTO6Oscilloscope::EnableChannel(size_t i)
{
	m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":STATE 1");

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelsEnabled[i] = true;
}

void RSRTO6Oscilloscope::DisableChannel(size_t i)
{
	m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":STATE 0");

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelsEnabled[i] = false;
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

// PROBE1:SETUP:ATT:MODE?
//  If MAN: PROBE1:SETUP:GAIN:MANUAL?
//  If AUTO: PROBE1:SETUP:ATT?

double RSRTO6Oscilloscope::GetChannelAttenuation(size_t i)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelAttenuations.find(i) != m_channelAttenuations.end())
			return m_channelAttenuations[i];
	}

	string reply;
	reply = m_transport->SendCommandQueuedWithReply(
						"PROBE" + to_string(i+1) + ":SETUP:ATT:MODE?");

	double attenuation;

	if (reply == "MAN")
	{
		reply = m_transport->SendCommandQueuedWithReply(
						"PROBE" + to_string(i+1) + ":SETUP:GAIN:MANUAL?");
		attenuation = stod(reply);
	}
	else
	{
		reply = m_transport->SendCommandQueuedWithReply(
						"PROBE" + to_string(i+1) + ":SETUP:ATT?");
		attenuation = stod(reply);
	}

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelAttenuations[i] = attenuation;
	return attenuation;
}

void RSRTO6Oscilloscope::SetChannelAttenuation(size_t i, double atten)
{
	string reply;
	reply = m_transport->SendCommandQueuedWithReply(
						"PROBE" + to_string(i+1) + ":SETUP:ATT:MODE?");

	if (reply == "MAN")
	{
		m_transport->SendCommandQueued(
						"PROBE" + to_string(i+1) + ":SETUP:GAIN:MANUAL " + to_string(atten));

		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_channelAttenuations[i] = atten;
	}
	else
	{
		// Can't override attenuation of known probe type
	}
}

std::string RSRTO6Oscilloscope::GetProbeName(size_t i)
{
	return m_transport->SendCommandQueuedWithReply(
						"PROBE" + to_string(i+1) + ":SETUP:NAME?");
}

unsigned int RSRTO6Oscilloscope::GetChannelBandwidthLimit(size_t i)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelBandwidthLimits.find(i) != m_channelBandwidthLimits.end())
			return m_channelBandwidthLimits[i];
	}

	string reply;
	reply = m_transport->SendCommandQueuedWithReply(m_channels[i]->GetHwname() + ":BANDWIDTH?");

	unsigned int bw = 0;

	if (reply == "FULL")
	{
		bw = 0;
	}
	else if (reply == "B200")
	{
		bw = 200;
	}
	else if (reply == "B20")
	{
		bw = 20;
	}
	else
	{
		LogWarning("Unknown reported bandwidth: %s\n", reply.c_str());
	}

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelBandwidthLimits[i] = bw;
	return bw;
}

void RSRTO6Oscilloscope::SetChannelBandwidthLimit(size_t i, unsigned int limit_mhz)
{
	LogDebug("Request bandwidth: %u\n", limit_mhz);

	string limit_str;

	if (limit_mhz == 0)
	{
		limit_str = "FULL";
		limit_mhz = 0;
	}
	else if (limit_mhz == 20)
	{
		limit_str = "B20";
		limit_mhz = 20;
	}
	else if (limit_mhz == 200)
	{
		limit_str = "B200";
		limit_mhz = 200;
	}
	else
	{
		LogWarning("Unsupported requested bandwidth\n");
		return;
	}

	m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":BANDWIDTH " + limit_str);

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelBandwidthLimits[i] = limit_mhz;
}

vector<unsigned int> RSRTO6Oscilloscope::GetChannelBandwidthLimiters(size_t /*i*/)
{
	vector<unsigned int> ret;
	ret.push_back(20);
	ret.push_back(200);
	ret.push_back(0);
	return ret;
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
	else
	{
		if (state != "1")
			LogWarning("ACQuire:CURRent? -> %s\n", state.c_str());

		m_triggerArmed = false;
		return TRIGGER_MODE_TRIGGERED;
	}

	// return m_triggerArmed ? TRIGGER_MODE_TRIGGERED : TRIGGER_MODE_STOP;
}

bool RSRTO6Oscilloscope::AcquireData()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_transport->FlushCommandQueue();
	LogDebug(" ** AcquireData ** \n");

	GetSampleDepth();

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
		string reply = m_secondSocket->SendCommandImmediateWithReply(m_channels[i]->GetHwname() + ":DATA:HEAD?; *WAI");

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

		size_t reported_srate = (FS_PER_SECOND / fs_per_sample);

		if (reported_srate != m_sampleRate)
		{
			LogWarning("Reported sample rate %lu != expected sample rate %lu; using what it said\n", reported_srate, m_sampleRate);
		}

		if (length != m_sampleDepth)
		{
			LogWarning("Reported depth %lu != expected depth %lu; using what I think is correct\n", length, m_sampleDepth);
			length = m_sampleDepth;
		}

		//Set up the capture we're going to store our data into (no high res timer on R&S scopes)
		auto cap = new UniformAnalogWaveform;
		cap->m_timescale = fs_per_sample;
		cap->m_triggerPhase = 0;
		cap->m_startTimestamp = time(NULL);
		double t = GetTime();
		cap->m_startFemtoseconds = (t - floor(t)) * FS_PER_SECOND;

		cap->Resize(length);
		cap->PrepareForCpuAccess();

		size_t transferred = 0;
		const size_t block_size = 50e6;
		// Chosen to match what works on my coworker's macbook :/

		LogDebug("Starting acquisition phase. length = %lu\n", length);

		unsigned char* dest_buf = (unsigned char*)cap->m_samples.GetCpuPointer();

		while (transferred != length)
		{
			size_t this_length = block_size;
			if (this_length > (length - transferred))
				this_length = length - transferred;

			string params =  " "+to_string(transferred)+","+to_string(this_length);

			if (transferred == 0 && this_length == length)
				params = "";

			//Ask for the data
			size_t len_bytes;
			unsigned char* samples = (unsigned char*)m_secondSocket->SendCommandImmediateWithRawBlockReply(m_channels[i]->GetHwname() + ":DATA?"+params+"; *WAI", len_bytes);

			if (len_bytes != (this_length*sizeof(float)))
			{
				LogFatal("Unexpected number of bytes back");
			}

			unsigned char* cpy_target = dest_buf+(transferred*sizeof(float));
			LogDebug("Copying %luB from %p to %p\n", len_bytes, samples, cpy_target);

			memcpy(cpy_target, samples, len_bytes);
			transferred += this_length;
			delete[] samples;

			//Discard trailing newline
			uint8_t disregard;
			m_secondSocket->ReadRawData(1, &disregard);
		}

		cap->MarkSamplesModifiedFromCpu();

	

		//Done, update the data
		pending_waveforms[i].push_back(cap);
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
		m_secondSocket->SendCommandImmediate("SINGle");
		usleep(100000);
		// If we don't wait here, sending the query for available waveforms will race and return 1 for the exitisting waveform and jam everything up.
		m_triggerArmed = true;
	}
	else
	{
		m_triggerArmed = false;
	}

	auto end_time = std::chrono::system_clock::now();

	LogDebug("Acquisition took %lu\n", std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count());

	return any_data;
}

void RSRTO6Oscilloscope::Start()
{
	// lock_guard<recursive_mutex> lock(m_mutex);
	LogDebug("Start\n");
	m_secondSocket->SendCommandImmediate("SINGle");
	usleep(100000);
	// If we don't wait here, sending the query for available waveforms will race and return 1 for the exitisting waveform and jam everything up.
	m_triggerArmed = true;
	m_triggerOneShot = false;
}

void RSRTO6Oscilloscope::StartSingleTrigger()
{
	// lock_guard<recursive_mutex> lock(m_mutex);
	LogDebug("Start oneshot\n");
	m_secondSocket->SendCommandImmediate("SINGle");
	usleep(100000);
	// If we don't wait here, sending the query for available waveforms will race and return 1 for the exitisting waveform and jam everything up.
	m_triggerArmed = true;
	m_triggerOneShot = true;
}

void RSRTO6Oscilloscope::Stop()
{
	m_triggerArmed = false;

	// lock_guard<recursive_mutex> lock(m_mutex);
	LogDebug("Stop!\n");
	m_secondSocket->SendCommandImmediate("STOP");
	m_triggerArmed = false;
	m_triggerOneShot = true;
}

void RSRTO6Oscilloscope::ForceTrigger()
{
	if (m_triggerArmed)
		m_transport->SendCommandImmediate("TRIGGER1:FORCE");
}

bool RSRTO6Oscilloscope::IsTriggerArmed()
{
	return m_triggerArmed;
}

vector<uint64_t> RSRTO6Oscilloscope::GetSampleRatesNonInterleaved()
{
	LogWarning("RSRTO6Oscilloscope::GetSampleRatesNonInterleaved unimplemented\n");

	// FIXME -- Arbitrarily copied from Tek
	vector<uint64_t> ret;

	const int64_t k = 1000;
	const int64_t m = k*k;
	const int64_t g = k*m;

	uint64_t bases[] = { 1000, 1250, 2500, 3125, 5000, 6250 };
	vector<uint64_t> scales = {1, 10, 100, 1*k};

	for(auto b : bases)
		ret.push_back(b / 10);

	for(auto scale : scales)
	{
		for(auto b : bases)
			ret.push_back(b * scale);
	}

	// // MSO6 also supports these, or at least had them available in the picker before.
	// // TODO: Are these actually supported?

	// if (m_family == FAMILY_MSO6) {
	// 	for(auto b : bases) {
	// 		ret.push_back(b * 10 * k);
	// 	}
	// }

	// We break with the pattern on the upper end of the frequency range
	ret.push_back(12500 * k);
	ret.push_back(25 * m);
	ret.push_back(31250 * k);
	ret.push_back(62500 * k);
	ret.push_back(125 * m);
	ret.push_back(250 * m);
	ret.push_back(312500 * k);
	ret.push_back(625 * m);
	ret.push_back(1250 * m);
	ret.push_back(1562500 * k);
	ret.push_back(3125 * m);
	ret.push_back(6250 * m);
	ret.push_back(12500 * m);

	// Below are interpolated. 8 bits, not 12.
	//TODO: we can save bandwidth by using 8 bit waveform download for these

	ret.push_back(25 * g);

	// MSO5 supports these, TODO: Does MSO6?
	ret.push_back(25000 * m);
	ret.push_back(62500 * m);
	ret.push_back(125000 * m);
	ret.push_back(250000 * m);
	ret.push_back(500000 * m);

	return ret;
}

vector<uint64_t> RSRTO6Oscilloscope::GetSampleRatesInterleaved()
{
	return GetSampleRatesNonInterleaved();
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

	//FIXME -- Arbitrarily copied from Tek
	vector<uint64_t> ret;

	const int64_t k = 1000;
	const int64_t m = k*k;
	// const int64_t g = k*m;

	ret.push_back(500);
	ret.push_back(1 * k);
	ret.push_back(2 * k);
	ret.push_back(5 * k);
	ret.push_back(10 * k);
	ret.push_back(20 * k);
	ret.push_back(50 * k);
	ret.push_back(100 * k);
	ret.push_back(200 * k);
	ret.push_back(500 * k);

	ret.push_back(1 * m);
	ret.push_back(2 * m);
	ret.push_back(5 * m);
	ret.push_back(10 * m);
	ret.push_back(20 * m);
	ret.push_back(50 * m);
	ret.push_back(62500 * k);
	ret.push_back(100 * m);
	ret.push_back(125 * m);

	return ret;
}

vector<uint64_t> RSRTO6Oscilloscope::GetSampleDepthsInterleaved()
{
	return GetSampleRatesNonInterleaved();
}

uint64_t RSRTO6Oscilloscope::GetSampleRate()
{
	if(m_sampleRateValid)
		return m_sampleRate;

	m_sampleRate = stod(m_transport->SendCommandQueuedWithReply("ACQUIRE:SRATE?"));
	m_sampleRateValid = true;

	return 1;
}

uint64_t RSRTO6Oscilloscope::GetSampleDepth()
{
	if(m_sampleDepthValid)
		return m_sampleDepth;

	GetSampleRate();

	m_sampleDepth = stod(m_transport->SendCommandQueuedWithReply("TIMEBASE:RANGE?")) * (double)m_sampleRate;
	m_sampleDepthValid = true;

	return 1;
}

void RSRTO6Oscilloscope::SetSampleDepth(uint64_t depth)
{
	GetSampleRate();

	//Update the cache
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_sampleDepth = depth;
		m_sampleDepthValid = true;
	}

	m_transport->SendCommandQueued(string("TIMEBASE:RANGE ") + to_string((double)depth / (double)m_sampleRate));
}

void RSRTO6Oscilloscope::SetSampleRate(uint64_t rate)
{
	//Update the cache
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_sampleRate = rate;
		m_sampleRateValid = true;
	}

	m_transport->SendCommandQueued(string("ACQUIRE:SRATE ") + to_string(rate));

	SetSampleDepth(m_sampleDepth);
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
	if (m_trigger == NULL)
	{
		m_trigger = new EdgeTrigger(this);
		EdgeTrigger* et = dynamic_cast<EdgeTrigger*>(m_trigger);

		et->SetType(EdgeTrigger::EDGE_RISING);
		et->SetInput(0, StreamDescriptor(GetChannelByHwName("CHAN1"), 0), true);
		et->SetLevel(1.0);
		PushTrigger();
	}
}

/**
	@brief Reads settings for an edge trigger from the instrument
 */
void RSRTO6Oscilloscope::PullEdgeTrigger()
{
	// TODO
}

void RSRTO6Oscilloscope::PushTrigger()
{
	auto et = dynamic_cast<EdgeTrigger*>(m_trigger);
	if(et)
		PushEdgeTrigger(et);
	else
		LogWarning("Unknown trigger type (not an edge)\n");
}

/**
	@brief Pushes settings for an edge trigger to the instrument
 */
void RSRTO6Oscilloscope::PushEdgeTrigger(EdgeTrigger* trig)
{
	m_transport->SendCommandQueued("TRIGGER1:EVENT SINGLE");
	m_transport->SendCommandQueued("TRIGGER1:TYPE EDGE");
	m_transport->SendCommandQueued(string("TRIGGER1:SOURCE ") + trig->GetInput(0).m_channel->GetHwname());

	switch(trig->GetType())
	{
		case EdgeTrigger::EDGE_RISING:
			m_transport->SendCommandQueued("TRIGGER1:EDGE:SLOPE POSITIVE");
			break;

		case EdgeTrigger::EDGE_FALLING:
			m_transport->SendCommandQueued("TRIGGER1:EDGE:SLOPE NEGATIVE");
			break;

		case EdgeTrigger::EDGE_ANY:
			m_transport->SendCommandQueued("TRIGGER1:EDGE:SLOPE EITHER");
			break;

		default:
			LogWarning("Unknown edge type\n");
			break;
	}

	m_transport->SendCommandQueued(string("TRIGGER1:LEVEL") /*+ to_string(trig->GetInput(0).m_channel->GetIndex())*/ + " " + to_string(trig->GetLevel()));
}
