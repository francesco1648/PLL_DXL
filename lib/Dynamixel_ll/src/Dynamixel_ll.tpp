#pragma once


template <uint8_t N>
uint8_t DynamixelLL::setOperatingMode(const uint8_t (&modes)[N])
{
    // Check if array size N matches the expected number of motors
    if (checkArraySize(N) != 0)
        return 1; // Propagate error code

    uint32_t processedModes[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        // Allowed: 1 = Velocity, 3 = Position, 4 = Extended Position, 16 = PWM.
        if (!(modes[i] == 1 || modes[i] == 3 || modes[i] == 4 || modes[i] == 16))
        {
            if (_debug)
                Serial.print("Error: Unsupported operating mode.");
            return 1; // error: unsupported mode is provided
        }
        processedModes[i] = modes[i];
    }
    return writeRegister(11, processedModes, 1); // EEPROM address 11, 1 byte
}


template <uint8_t N>
uint8_t DynamixelLL::setHomingOffset(const float (&offsetAngle)[N])
{
    // Check if array size N matches the expected number of motors
    if (checkArraySize(N) != 0)
        return 1; // Propagate error code

    int32_t offsetPulse[_numMotors];
    uint32_t offsetArray[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        // Convert angle in degrees to pulses using conversion factor 0.088 [deg/pulse].
        offsetPulse[i] = static_cast<int32_t>(offsetAngle[i] / 0.088);
        if (offsetPulse[i] > 1044479)
        {
            offsetPulse[i] = 1044479;
            if (_debug)
                Serial.println("Warning: Homing offset clamped to 1044479.");
        } else if (offsetPulse[i] < -1044479) {
            offsetPulse[i] = -1044479;
            if (_debug)
                Serial.println("Warning: Homing offset clamped to -1044479.");
        }
        // Convert to uint32_t for writing to register.
        offsetArray[i] = static_cast<uint32_t>(offsetPulse[i]);
    }
    return writeRegister(20, offsetArray, 4); // RAM address 20, 4 bytes
}


template <uint8_t N>
uint8_t DynamixelLL::setGoalPosition_PCM(const uint16_t (&goalPositions)[N])
{
    // Check if array size N matches the expected number of motors
    if (checkArraySize(N) != 0)
        return 1; // Propagate error code

    uint32_t processedPositions[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        processedPositions[i] = goalPositions[i];

        // Note: The servo internally also limits the value;
        // the main purpose of the clamping here is to aid debugging.
        if (processedPositions[i] > 4095) {
            processedPositions[i] = 4095;
            if (_debug)
                Serial.println("Warning: Goal position clamped to 4095.");
        }
    }
    return writeRegister(116, processedPositions, 4); // RAM address 116, 4 bytes
}


template <uint8_t N>
uint8_t DynamixelLL::setGoalPosition_A_PCM(const float (&angleDegrees)[N])
{
    // Check if array size N matches the expected number of motors
    if (checkArraySize(N) != 0)
        return 1; // Propagate error code

    uint32_t processedPositions[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        // Convert angle in degrees to pulses using conversion factor 0.088 [deg/pulse].
        processedPositions[i] = static_cast<uint32_t>(angleDegrees[i] / 0.088);
        if (processedPositions[i] > 4095) {
            processedPositions[i] = 4095;
            if (_debug)
                Serial.println("Warning: Angle conversion resulted in value exceeding 4095, clamped.");
        }
    }
    return writeRegister(116, processedPositions, 4); // RAM address 116, 4 bytes
}


template <uint8_t N>
uint8_t DynamixelLL::setGoalPosition_EPCM(int32_t (&extendedPositions)[N])
{
    // Check if N matches the expected number of motors
    if (checkArraySize(N) != 0)
        return 1; // Propagate error code

    uint32_t processedPositions[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        // Clamp within valid range: -1,048,575 to +1,048,575 pulses.
        if (extendedPositions[i] > 1048575)
        {
            extendedPositions[i] = 1048575;
            if (_debug)
                Serial.println("Warning: Extended position clamped to 1048575.");
        } else if (extendedPositions[i] < -1048575) {
            extendedPositions[i] = -1048575;
            if (_debug)
                Serial.println("Warning: Extended position clamped to -1048575.");
        }
        // Convert to uint32_t for writing to register.
        processedPositions[i] = static_cast<uint32_t>(extendedPositions[i]);
    }
    return writeRegister(116, processedPositions, 4); // RAM address 116, 4 bytes
}


template <uint8_t N>
uint8_t DynamixelLL::setTorqueEnable(const bool (&enable)[N])
{
    // Check if array size N matches the expected number of motors
    if (checkArraySize(N) != 0)
        return 1; // Propagate error code

    uint32_t processedValues[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
        processedValues[i] = enable[i] ? 1 : 0;
    return writeRegister(64, processedValues, 1); // RAM address 64, 1 byte
}


template <uint8_t N>
uint8_t DynamixelLL::setLED(const bool (&enable)[N])
{
    // Check if array size N matches the expected number of motors
    if (checkArraySize(N) != 0)
        return 1; // Propagate error code

    uint32_t processedValues[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
        processedValues[i] = enable[i] ? 1 : 0;
    return writeRegister(65, processedValues, 1); // RAM address 65, 1 byte
}


template <uint8_t N>
uint8_t DynamixelLL::setStatusReturnLevel(const uint8_t (&levels)[N])
{
    // Check if array size N matches the expected number of motors
    if (checkArraySize(N) != 0)
        return 1; // Propagate error code

    uint32_t processedLevels[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        // Valid status return levels are 0, 1, or 2.
        if (levels[i] > 2)
        {
            if (_debug)
                Serial.println("Error: Invalid status return level. Allowed values: 0, 1, or 2.");
            return 1; // Error code for invalid status return level.
        }
        processedLevels[i] = levels[i];
    }
    return writeRegister(68, processedLevels, 1); // RAM address 68, 1 byte
}


template <uint8_t N>
uint8_t DynamixelLL::setID(const uint8_t (&newIDs)[N])
{
    // Check if array size N matches the expected number of motors
    if (checkArraySize(N) != 0)
        return 1; // Propagate error code

    uint32_t processedIDs[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        // Valid IDs are 0 to 253; 254 is reserved for Broadcast.
        if (newIDs[i] > 253)
        {
            if (_debug)
                Serial.println("Error: Invalid ID. Valid IDs are 0 to 253.");
            return 1; // Error code for invalid ID.
        }
        processedIDs[i] = newIDs[i];
    }
    return writeRegister(7, newIDs, 1); // EEPROM address 7, 1 byte
}


template <uint8_t N>
uint8_t DynamixelLL::setBaudRate(const uint8_t (&baudRates)[N])
{
    // Check if array size N matches the expected number of motors
    if (checkArraySize(N) != 0)
        return 1; // Propagate error code

    uint32_t processedBaudRates[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        const uint8_t allowed[] = {0, 1, 2, 3, 4, 5, 6, 7};
        bool valid = false;

        // Check that the provided baudRate code is within the allowed set.
        for (uint8_t j = 0; j < sizeof(allowed) / sizeof(allowed[0]); j++)
        {
            if (allowed[j] == baudRates[i])
            {
                valid = true;
                break;
            }
        }

        // If the baud rate code is not valid, output a debug message and return an error code.
        if (!valid)
        {
            if (_debug)
            {
                Serial.print("Error: Unrecognized baud rate code: ");
                Serial.println(baudRates[i]);
            }
            return 1;
        }
        processedBaudRates[i] = baudRates[i];
    }
    return writeRegister(8, processedBaudRates, 1); // EEPROM address 8, 1 byte
}


template <uint8_t N>
uint8_t DynamixelLL::setReturnDelayTime(const uint8_t (&delayTime)[N])
{
    // Check if array size N matches the expected number of motors
    if (checkArraySize(N) != 0)
        return 1; // Propagate error code

    uint32_t processedDelayTime[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        processedDelayTime[i] = delayTime[i];
        // If the provided delayTime exceeds the maximum value, clamp it to 254.
        if (processedDelayTime[i] > 254)
        {
            processedDelayTime[i] = 254;
            if (_debug)
                Serial.println("Warning: setReturnDelayTime clamped to 254.");
        }
    }
    return writeRegister(9, processedDelayTime, 1); // EEPROM address 9, 1 byte
}


template <uint8_t N>
uint8_t DynamixelLL::setDriveMode(const bool (&torqueOnByGoalUpdate)[N],
                                  const bool (&timeBasedProfile)[N],
                                  const bool (&reverseMode)[N])
{
    // Check if array size N matches the expected number of motors
    if (checkArraySize(N) != 0)
        return 1; // Propagate error code

    uint32_t processedDriveModes[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        uint8_t mode = 0;
        if (torqueOnByGoalUpdate[i])
            mode |= 0x08; // Set Bit 3.
        if (timeBasedProfile[i])
            mode |= 0x04; // Set Bit 2.
        if (reverseMode[i])
            mode |= 0x01; // Set Bit 0.
        processedDriveModes[i] = mode;
    }
    return writeRegister(10, processedDriveModes, 1); // EEPROM address 10, 1 byte
}


template <uint8_t N>
uint8_t DynamixelLL::setProfileVelocity(const uint32_t (&profileVelocity)[N])
{
    // Check if array size N matches the expected number of motors
    if (checkArraySize(N) != 0)
        return 1; // Propagate error code

    uint32_t processedProfileVelocity[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        uint32_t driveModeTemp = 0;
        uint8_t error = readRegister(10, driveModeTemp, 1);
        uint8_t driveMode = driveModeTemp & 0xFF;
        // Bit 2 (0x04) set indicates time-based profile.
        bool timeBased = (error == 0) && ((driveMode & 0x04) != 0);

        // Select maximum allowed velocity based on profile type.
        // For time-based profiles, max = 32737; for velocity-based, max = 32767.
        const uint32_t maxProfileVelocity = timeBased ? 32737UL : 32767UL;

        // Clamp the input value if it exceeds the allowed maximum.
        if (profileVelocity > maxProfileVelocity)
        {
            if (_debug)
            {
                Serial.print("Profile velocity clamped to ");
                Serial.println(maxProfileVelocity);
            }
            processedProfileVelocity[i] = maxProfileVelocity;
        } else
            processedProfileVelocity[i] = profileVelocity[i];
    }
    return writeRegister(112, processedProfileVelocity, 4); // RAM address 112, 4 bytes
}


template <uint8_t N>
uint8_t DynamixelLL::setProfileAcceleration(const uint32_t (&profileAcceleration)[N])
{
    // Check if array size N matches the expected number of motors
    if (checkArraySize(N) != 0)
        return 1; // Propagate error code

    uint32_t processedProfileAcceleration[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        // Read drive mode from register 10 to determine if a time-based profile is active.
        uint32_t driveModeTemp = 0;
        uint8_t error = readRegister(10, driveModeTemp, 1);
        uint8_t driveMode = driveModeTemp & 0xFF;
        bool timeBased = (error == 0) && ((driveMode & 0x04) != 0);

        // Choose the maximum allowed acceleration based on profile type.
        // For time-based profiles, maximum is 32737; otherwise, 32767.
        const uint32_t maxProfileAcceleration = timeBased ? 32737UL : 32767UL;

        // Clamp the input acceleration if it exceeds this maximum.
        if (profileAcceleration > maxProfileAcceleration)
        {
            if (_debug)
            {
                Serial.print("Profile acceleration clamped to ");
                Serial.println(maxProfileAcceleration);
            }
            processedProfileAcceleration[i] = maxProfileAcceleration;
        } else
            processedProfileAcceleration[i] = profileAcceleration[i];

        // For time-based profiles, ensure that acceleration does not exceed half of the current profile velocity.
        uint32_t currentProfileVelocity = 0;
        error = readRegister(112, currentProfileVelocity, 4);
        if (timeBased && error == 0 && currentProfileVelocity > 0 && profileAcceleration > (currentProfileVelocity / 2))
        {
            uint32_t clampedValue = currentProfileVelocity / 2;
            if (_debug)
            {
                Serial.print("Profile acceleration clamped to half of current profile velocity: ");
                Serial.println(clampedValue);
            }
            processedProfileAcceleration[i] = clampedValue;
        } else if (error != 0 && _debug) {
            Serial.print("Error reading Profile Velocity: ");
            Serial.println(error);
        } else
            processedProfileAcceleration[i] = profileAcceleration[i];
    }
    return writeRegister(108, processedProfileAcceleration, 4); // RAM address 108, 4 bytes
}


template <uint8_t N>
uint8_t DynamixelLL::getPresentPosition(int32_t (&presentPositions)[N])
{
    // Check if array size N matches the expected number of motors
    if (checkArraySize(N) != 0)
        return 1; // Propagate error code

    uint32_t temp[_numMotors];
    uint8_t error = syncRead(132, 4, _motorIDs, temp, _numMotors); // RAM address 132, 4 bytes
    if (error != 0)
    {
        if (_debug)
        {
            Serial.print("Error reading present position: ");
            Serial.println(error);
        }
    } else
    {
        for (uint8_t i = 0; i < _numMotors; i++)
            presentPositions[i] = static_cast<int32_t>(temp[i]);
    }
    return error;
}