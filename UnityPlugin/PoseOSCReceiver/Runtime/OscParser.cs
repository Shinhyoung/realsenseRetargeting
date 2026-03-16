// ──────────────────────────────────────────────
// OscParser.cs — 경량 OSC 1.0 바이너리 파서
// 외부 라이브러리 의존성 없음
// ──────────────────────────────────────────────
using System;
using System.Collections.Generic;
using System.Text;

namespace PoseOSCReceiver
{
    public struct OscMessage
    {
        public string Address;
        public float[] Floats;
    }

    public static class OscParser
    {
        /// <summary>
        /// 수신된 UDP 바이트 배열을 파싱하여 OscMessage 리스트로 반환.
        /// Bundle과 단일 Message 모두 지원.
        /// </summary>
        public static List<OscMessage> Parse(byte[] data, int length)
        {
            var messages = new List<OscMessage>();
            if (length < 4) return messages;

            // Bundle 여부 확인: "#bundle\0"
            if (data[0] == (byte)'#')
            {
                ParseBundle(data, 0, length, messages);
            }
            else
            {
                var msg = ParseMessage(data, 0, length);
                if (msg.HasValue) messages.Add(msg.Value);
            }

            return messages;
        }

        static void ParseBundle(byte[] data, int offset, int end, List<OscMessage> messages)
        {
            // "#bundle\0" (8 bytes) + timetag (8 bytes) = 16 bytes header
            int pos = offset + 16;

            while (pos < end)
            {
                if (pos + 4 > end) break;

                int size = ReadInt32(data, pos);
                pos += 4;

                if (size <= 0 || pos + size > end) break;

                // 중첩 번들 확인
                if (data[pos] == (byte)'#')
                {
                    ParseBundle(data, pos, pos + size, messages);
                }
                else
                {
                    var msg = ParseMessage(data, pos, pos + size);
                    if (msg.HasValue) messages.Add(msg.Value);
                }

                pos += size;
            }
        }

        static OscMessage? ParseMessage(byte[] data, int offset, int end)
        {
            // Address string (null-terminated, 4-byte aligned)
            int addrEnd = FindNull(data, offset, end);
            if (addrEnd < 0) return null;

            string address = Encoding.ASCII.GetString(data, offset, addrEnd - offset);
            int pos = Align4(addrEnd + 1);

            // Type tag string: ",fff...\0"
            if (pos >= end || data[pos] != (byte)',') return null;

            int tagStart = pos + 1; // skip ','
            int tagEnd = FindNull(data, tagStart, end);
            if (tagEnd < 0) return null;

            int tagCount = tagEnd - tagStart;
            pos = Align4(tagEnd + 1);

            // Float 인수만 추출 (이 프로젝트에서는 float만 사용)
            var floats = new List<float>();
            for (int i = 0; i < tagCount; i++)
            {
                char tag = (char)data[tagStart + i];
                switch (tag)
                {
                    case 'f':
                        if (pos + 4 > end) return null;
                        floats.Add(ReadFloat(data, pos));
                        pos += 4;
                        break;
                    case 'i':
                        pos += 4; // skip int
                        break;
                    case 's':
                        int sEnd = FindNull(data, pos, end);
                        pos = sEnd >= 0 ? Align4(sEnd + 1) : end;
                        break;
                    case 'b':
                        if (pos + 4 > end) return null;
                        int blobSize = ReadInt32(data, pos);
                        pos += 4 + Align4Size(blobSize);
                        break;
                    default:
                        break;
                }
            }

            return new OscMessage
            {
                Address = address,
                Floats = floats.ToArray()
            };
        }

        // ── 유틸리티 ──

        static int FindNull(byte[] data, int from, int end)
        {
            for (int i = from; i < end; i++)
                if (data[i] == 0) return i;
            return -1;
        }

        static int Align4(int pos) => (pos + 3) & ~3;
        static int Align4Size(int size) => (size + 3) & ~3;

        static int ReadInt32(byte[] data, int offset)
        {
            // OSC uses big-endian
            return (data[offset] << 24)
                 | (data[offset + 1] << 16)
                 | (data[offset + 2] << 8)
                 | data[offset + 3];
        }

        static float ReadFloat(byte[] data, int offset)
        {
            // Big-endian → little-endian conversion
            byte[] tmp = new byte[4];
            tmp[0] = data[offset + 3];
            tmp[1] = data[offset + 2];
            tmp[2] = data[offset + 1];
            tmp[3] = data[offset];
            return BitConverter.ToSingle(tmp, 0);
        }
    }
}
