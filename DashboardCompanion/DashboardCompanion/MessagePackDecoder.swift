import Foundation

enum MessagePackValue {
    case double(Double)
    case bool(Bool)
    case string(String)
    case array([MessagePackValue])
    case map([String: MessagePackValue])
    case null
}

enum MessagePackDecoder {
    static func decodeTopLevelValues(_ data: Data) -> [MessagePackValue] {
        var parser = Parser(data: data)
        var values: [MessagePackValue] = []
        while !parser.isAtEnd {
            if let value = parser.readValue() {
                values.append(value)
            } else {
                break
            }
        }
        return values
    }

    private struct Parser {
        let data: Data
        var offset = 0

        private static let maxContainerElements = 4_096
        private static let maxStringLength = 65_536

        var isAtEnd: Bool { offset >= data.count }
        var remainingBytes: Int { data.count - offset }

        mutating func readValue() -> MessagePackValue? {
            guard let byte = readUInt8() else { return nil }

            switch byte {
            case 0x00 ... 0x7f:
                return .double(Double(byte))
            case 0x80 ... 0x8f:
                return readMap(count: Int(byte & 0x0f))
            case 0x90 ... 0x9f:
                return readArray(count: Int(byte & 0x0f))
            case 0xa0 ... 0xbf:
                return .string(readString(length: Int(byte & 0x1f)))
            case 0xc0:
                return .null
            case 0xc2:
                return .bool(false)
            case 0xc3:
                return .bool(true)
            case 0xc4:
                guard let length = readUInt8() else { return nil }
                return skipBlob(length: Int(length))
            case 0xc5:
                guard let length = readUInt16() else { return nil }
                return skipBlob(length: Int(length))
            case 0xc6:
                guard let length = readUInt32() else { return nil }
                return skipBlob(length: Int(length))
            case 0xca:
                guard let value = readFloat32() else { return nil }
                return .double(Double(value))
            case 0xcb:
                guard let value = readFloat64() else { return nil }
                return .double(value)
            case 0xcc:
                guard let value = readUInt8() else { return nil }
                return .double(Double(value))
            case 0xcd:
                guard let value = readUInt16() else { return nil }
                return .double(Double(value))
            case 0xce:
                guard let value = readUInt32() else { return nil }
                return .double(Double(value))
            case 0xcf:
                guard let value = readUInt64() else { return nil }
                return .double(Double(value))
            case 0xd0:
                guard let value = readInt8() else { return nil }
                return .double(Double(value))
            case 0xd1:
                guard let value = readInt16() else { return nil }
                return .double(Double(value))
            case 0xd2:
                guard let value = readInt32() else { return nil }
                return .double(Double(value))
            case 0xd3:
                guard let value = readInt64() else { return nil }
                return .double(Double(value))
            case 0xd4:
                return skipExtension(length: 1)
            case 0xd5:
                return skipExtension(length: 2)
            case 0xd6:
                return skipExtension(length: 4)
            case 0xd7:
                return skipExtension(length: 8)
            case 0xd8:
                return skipExtension(length: 16)
            case 0xd9:
                guard let length = readUInt8() else { return nil }
                return readStringValue(length: Int(length))
            case 0xda:
                guard let length = readUInt16() else { return nil }
                return readStringValue(length: Int(length))
            case 0xdb:
                guard let length = readUInt32() else { return nil }
                return readStringValue(length: Int(length))
            case 0xdc:
                guard let count = readUInt16() else { return nil }
                return readArray(count: Int(count))
            case 0xdd:
                guard let count = readUInt32() else { return nil }
                return readArray(count: Int(count))
            case 0xde:
                guard let count = readUInt16() else { return nil }
                return readMap(count: Int(count))
            case 0xdf:
                guard let count = readUInt32() else { return nil }
                return readMap(count: Int(count))
            case 0xc7:
                guard let length = readUInt8() else { return nil }
                return skipExtension(length: Int(length))
            case 0xc8:
                guard let length = readUInt16() else { return nil }
                return skipExtension(length: Int(length))
            case 0xc9:
                guard let length = readUInt32() else { return nil }
                return skipExtension(length: Int(length))
            case 0xe0 ... 0xff:
                return .double(Double(Int8(bitPattern: byte)))
            default:
                return nil
            }
        }

        mutating func readArray(count: Int) -> MessagePackValue? {
            guard isReasonableContainerCount(count) else { return nil }
            var result: [MessagePackValue] = []
            result.reserveCapacity(count)
            for _ in 0 ..< count {
                if let value = readValue() {
                    result.append(value)
                } else {
                    return nil
                }
            }
            return .array(result)
        }

        mutating func readMap(count: Int) -> MessagePackValue? {
            guard isReasonableContainerCount(count) else { return nil }
            var result: [String: MessagePackValue] = [:]
            result.reserveCapacity(count)
            for _ in 0 ..< count {
                guard let keyValue = readValue(),
                      case let .string(key) = keyValue,
                      let value = readValue()
                else { return nil }
                result[key] = value
            }
            return .map(result)
        }

        mutating func readStringValue(length: Int) -> MessagePackValue? {
            guard length >= 0,
                  length <= Self.maxStringLength,
                  length <= remainingBytes
            else { return nil }
            return .string(readString(length: length))
        }

        mutating func readString(length: Int) -> String {
            guard offset + length <= data.count else { return "" }
            let slice = data.subdata(in: offset ..< offset + length)
            offset += length
            return String(decoding: slice, as: UTF8.self)
        }

        mutating func skipBlob(length: Int) -> MessagePackValue? {
            guard length >= 0, length <= remainingBytes else { return nil }
            offset += length
            return .null
        }

        mutating func skipExtension(length: Int) -> MessagePackValue? {
            guard readInt8() != nil else { return nil }
            return skipBlob(length: length)
        }

        func isReasonableContainerCount(_ count: Int) -> Bool {
            guard count >= 0, count <= Self.maxContainerElements else { return false }
            return count <= remainingBytes
        }

        mutating func readUInt8() -> UInt8? {
            guard offset + 1 <= data.count else { return nil }
            defer { offset += 1 }
            return data[offset]
        }

        mutating func readUInt16() -> UInt16? {
            readInteger()
        }

        mutating func readUInt32() -> UInt32? {
            readInteger()
        }

        mutating func readUInt64() -> UInt64? {
            readInteger()
        }

        mutating func readInt8() -> Int8? {
            guard let value = readUInt8() else { return nil }
            return Int8(bitPattern: value)
        }

        mutating func readInt16() -> Int16? {
            readInteger()
        }

        mutating func readInt32() -> Int32? {
            readInteger()
        }

        mutating func readInt64() -> Int64? {
            readInteger()
        }

        mutating func readFloat32() -> Float32? {
            readInteger()
        }

        mutating func readFloat64() -> Float64? {
            readInteger()
        }

        mutating func readInteger<T>() -> T? where T: FixedWidthInteger {
            let size = MemoryLayout<T>.size
            guard offset + size <= data.count else { return nil }
            var raw: T = 0
            _ = withUnsafeMutableBytes(of: &raw) { destination in
                data.copyBytes(to: destination, from: offset ..< offset + size)
            }
            offset += size
            return raw.bigEndian
        }

        mutating func readInteger<T>() -> T? where T: BinaryFloatingPoint {
            let size = MemoryLayout<T>.size
            guard offset + size <= data.count else { return nil }

            let value: T
            if T.self == Float32.self {
                var bits: UInt32 = 0
                _ = withUnsafeMutableBytes(of: &bits) { destination in
                    data.copyBytes(to: destination, from: offset ..< offset + size)
                }
                value = Float32(bitPattern: bits.bigEndian) as! T
            } else {
                var bits: UInt64 = 0
                _ = withUnsafeMutableBytes(of: &bits) { destination in
                    data.copyBytes(to: destination, from: offset ..< offset + size)
                }
                value = Float64(bitPattern: bits.bigEndian) as! T
            }
            offset += size
            return value
        }
    }
}
