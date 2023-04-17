/*

Name:   catena-message-0x2f-port-1-decoder-ttn.js

Function:
        Decode MCCI port 0x01 messages for TTN console.

Copyright and License:
        See accompanying LICENSE file

Author:
        Pranau, MCCI Corporation   April 2023

*/

function DecodeU16(Parse) {
    var i = Parse.i;
    var bytes = Parse.bytes;
    var raw = (bytes[i] << 8) + bytes[i + 1];
    Parse.i = i + 2;
    return raw;
}

function DecodeI16(Parse) {
    var Vraw = DecodeU16(Parse);

    // interpret uint16 as an int16 instead.
    if (Vraw & 0x8000)
        Vraw += -0x10000;

    return Vraw;
}

function DecodeV(Parse) {
    return DecodeI16(Parse) / 4096.0;
}

/*

Name:  Decoder()

Function:
    Decode an MCCI Catena port-1 message for The Things Network console.

Definition:
    function Decoder(bytes, port) -> object

Description:
    This function decodes the message given by the byte array `bytes[]`,
    and returns an object with values in engineering units.

Returns:
    Object, or null if the bytes could not be decoded.

*/

function Decoder(bytes, port) {
    // Decode an uplink message from a buffer
    // (array) of bytes to an object of fields.
    var decoded = {};

    if (! (port === 1))
        return null;

    var uFormat = bytes[0];
    if (! (uFormat === 0x2f))
        return null;

    // an object to help us parse.
    var Parse = {};
    Parse.bytes = bytes;
    // i is used as the index into the message. Start with the flag byte.
    Parse.i = 1;

    // fetch the bitmap.
    var flags = bytes[Parse.i++];

    if (flags & 0x1) {
        decoded.vBat = DecodeV(Parse);
    }

    if (flags & 0x2) {
        decoded.vBus = DecodeV(Parse);
    }

    if (flags & 0x4) {
        var iBoot = bytes[Parse.i++];
        decoded.boot = iBoot;
    }    

    if (flags & 0x8) {
        // SAR Count Channel0
        decoded.sarCh0 = DecodeU16(Parse);
    }

    if (flags & 0x10) {
        // SAR Count Channel1
        decoded.sarCh1 = DecodeU16(Parse);
    }

    if (flags & 0x20) {
        // SAR Count Channel2
        decoded.sarCh2 = DecodeU16(Parse);
    }

    if (flags & 0x40) {
        // SAR Count Channel2
        decoded.amplitude = DecodeI16(Parse);
    }

    // at this point, decoded has the real values.
    return decoded;
}

// TTN V3 decoder
function decodeUplink(tInput) {
    var decoded = Decoder(tInput.bytes, tInput.fPort);
    var result = {};
    result.data = decoded;
    return result;
}