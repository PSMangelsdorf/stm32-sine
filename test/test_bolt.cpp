/*
 * This file is part of the stm32-sine project.
 *
 * Unit tests for the Bolt logic board (HW_BOLT) specific code.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <cmath>
#include <map>
#include "params.h"
#include "digio.h"
#include "anain.h"
#include "hwdefs.h"
#include "temp_meas.h"
#include "vehiclecontrol.h"
#include "test.h"

extern std::map<uint32_t, uint16_t> gpioSim;
extern uint16_t anaInSim[8];
//Channel numbers of the AnaIn stubs defined in test_vcu.cpp
static const int TMPHS_CHAN = 2, TMPM_CHAN = 3;

//ADC digits for the FS800 module NTC on the Bolt board: 4.7k pull-up to 3.3 V, 12 bit ADC
static uint16_t NtcDigits(float degC)
{
   float r = 5000.0f * expf(3433.0f * (1.0f / (degC + 273.15f) - 1.0f / 298.15f));
   return (uint16_t)lroundf(4095.0f * r / (r + 4700.0f));
}

static void TestNtcTablePoints()
{
   //Table points
   ASSERT(fabsf(TempMeas::Lookup(NtcDigits(25), TempMeas::TEMP_BOLT_FS800) - 25) < 0.2f);
   ASSERT(fabsf(TempMeas::Lookup(NtcDigits(100), TempMeas::TEMP_BOLT_FS800) - 100) < 0.2f);
   //Between table points: linear interpolation error stays small
   ASSERT(fabsf(TempMeas::Lookup(NtcDigits(87.3f), TempMeas::TEMP_BOLT_FS800) - 87.3f) < 0.5f);
   ASSERT(fabsf(TempMeas::Lookup(NtcDigits(-12.5f), TempMeas::TEMP_BOLT_FS800) + 12.5f) < 0.5f);
}

static void TestNtcTableLimits()
{
   //Open sensor (pulled to full scale) reads as the coldest value, shorted as the hottest
   ASSERT(TempMeas::Lookup(4095, TempMeas::TEMP_BOLT_FS800) == -40);
   ASSERT(TempMeas::Lookup(0, TempMeas::TEMP_BOLT_FS800) == 150);
}

static int MuxChannel()
{
   return DigIo::temp0_out.Get() + 2 * DigIo::temp1_out.Get();
}

static void TestTempMuxSequence()
{
   HWREV savedRev = hwRev;
   hwRev = HW_BOLT;
   DigIo::temp0_out.Configure(GPIOC, GPIO10, PinMode::OUTPUT);
   DigIo::temp1_out.Configure(GPIOC, GPIO8, PinMode::OUTPUT);
   DigIo::temp0_out.Clear();
   DigIo::temp1_out.Clear();

   //Mux walks 0 -> 1 -> 2 -> 0 and never selects the grounded channel 3
   VehicleControl::CalcAndOutputTemp();
   ASSERT(MuxChannel() == 1);
   VehicleControl::CalcAndOutputTemp();
   ASSERT(MuxChannel() == 2);
   VehicleControl::CalcAndOutputTemp();
   ASSERT(MuxChannel() == 0);

   DigIo::temp0_out.Configure(0, 0, PinMode::OUTPUT);
   DigIo::temp1_out.Configure(0, 0, PinMode::OUTPUT);
   gpioSim.clear();
   hwRev = savedRev;
}

static void TestTempMuxReadings()
{
   HWREV savedRev = hwRev;
   hwRev = HW_BOLT;
   DigIo::temp0_out.Configure(GPIOC, GPIO10, PinMode::OUTPUT);
   DigIo::temp1_out.Configure(GPIOC, GPIO8, PinMode::OUTPUT);
   DigIo::temp0_out.Clear();
   DigIo::temp1_out.Clear();
   Param::SetInt(Param::snsm, TempMeas::TEMP_KTY84);

   const float phaseTemps[3] = { 45, 72, 58 }; //U, V, W

   //Feed each mux channel its own reading for long enough for the output filters to settle
   for (int i = 0; i < 3000; i++)
   {
      int ch = MuxChannel();
      anaInSim[TMPHS_CHAN] = ch < 3 ? NtcDigits(phaseTemps[ch]) : 0;
      anaInSim[TMPM_CHAN] = ch == 0 ? 1600 : 0; //KTY84 table: 1600 digits = 100 degC
      VehicleControl::CalcAndOutputTemp();
   }

   //Heat sink temperature is the hottest of the three module NTCs
   ASSERT(fabsf(Param::GetFloat(Param::tmphs) - 72) < 0.5f);
   //Motor temperature comes from mux channel 0 with the user selected sensor type
   ASSERT(fabsf(Param::GetFloat(Param::tmpm) - 100) < 0.5f);

   anaInSim[TMPHS_CHAN] = anaInSim[TMPM_CHAN] = 0;
   DigIo::temp0_out.Configure(0, 0, PinMode::OUTPUT);
   DigIo::temp1_out.Configure(0, 0, PinMode::OUTPUT);
   gpioSim.clear();
   hwRev = savedRev;
}

static void TestGateOkValue()
{
   HWREV savedRev = hwRev;

   hwRev = HW_REV3;
   VehicleControl::GetDigInputs();
   ASSERT(Param::GetInt(Param::din_gateok) == 2); //not available on other boards

   hwRev = HW_BOLT;
   DigIo::gateok_in.Configure(GPIOA, GPIO0, PinMode::INPUT_FLT);
   gpio_set(GPIOA, GPIO0);
   VehicleControl::GetDigInputs();
   ASSERT(Param::GetInt(Param::din_gateok) == 1);
   gpio_clear(GPIOA, GPIO0);
   VehicleControl::GetDigInputs();
   ASSERT(Param::GetInt(Param::din_gateok) == 0);

   DigIo::gateok_in.Configure(0, 0, PinMode::INPUT_FLT);
   gpioSim.clear();
   hwRev = savedRev;
}

REGISTER_TEST(UnitTest, TestNtcTablePoints, TestNtcTableLimits, TestTempMuxSequence, TestTempMuxReadings,
              TestGateOkValue);
