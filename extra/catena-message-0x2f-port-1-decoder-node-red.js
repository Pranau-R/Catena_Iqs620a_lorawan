/*

Name:   catena-message-0x2f-port-1-decoder-node-red.js

Function:
        Decode MCCI port 0x01 messages for Node-Red flow.

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

// end of insertion of catena-message-0x2f-port-1-decoder-node-red.js

/*

Node-RED function body.

Input:
    msg     the object to be decoded.

            msg.payload_raw is taken
            as the raw payload if present; otheriwse msg.payload
            is taken to be a raw payload.

            msg.port is taken to be the LoRaWAN port nubmer.

Returns:
    This function returns a message body. It's a mutation of the
    input msg; msg.payload is changed to the decoded data, and
    msg.local is set to additional application-specific information.

*/

var bytes;

if ("payload_raw" in msg) {
    // the console already decoded this
    bytes = msg.payload_raw;  // pick up data for convenience
    // msg.payload_fields still has the decoded data from ttn
} else {
    // no console decode
    bytes = msg.payload;  // pick up data for conveneince
}

// try to decode.
var result = Decoder(bytes, msg.port);

if (result === null) {
    // not one of ours: report an error, return without a value,
    // so that Node-RED doesn't propagate the message any further.
    var eMsg = "not port 1/fmt 0x2f! port=" + msg.port.toString();
    if (port === 2) {
        if (Buffer.byteLength(bytes) > 0) {
            eMsg = eMsg + " fmt=" + bytes[0].toString();
        } else {
            eMsg = eMsg + " <no fmt byte>"
        }
    }
    node.error(eMsg);
    return;
}

// now update msg with the new payload and new .local field
// the old msg.payload is overwritten.
msg.payload = result;
msg.local =
    {
        nodeType: "CatenaIqs620a",
        platformType: "Catena 4610",
        radioType: "Murata",
        applicationName: "Touch and Proximity Sensor - LoRaWAN"
    };

return msg;