// SPDX-License-Identifier: Apache-2.0
using System.Net;
using System.Net.Sockets;
using System.Text;
using FluentAssertions;
using ImuStudio.Core.Mqtt;
using ImuStudio.Core.Recording;
using Xunit;

namespace ImuStudio.Core.Tests;

public class TransportTests
{
    private static void ReadExactly(Socket s, byte[] expected)
    {
        byte[] got = new byte[expected.Length];
        int n = 0;
        s.ReceiveTimeout = 2000;
        while (n < got.Length)
        {
            n += s.Receive(got, n, got.Length - n, SocketFlags.None);
        }
        got.Should().Equal(expected);
    }

    [Fact]
    public void Endpoint_answers_and_hands_over_what_is_published()
    {
        using MqttEndpoint endpoint = new(0, IPAddress.Loopback);
        List<(string Topic, byte[] Payload)> received = new();
        using ManualResetEventSlim two = new();
        endpoint.Published += (topic, payload) =>
        {
            lock (received)
            {
                received.Add((topic, payload));
                if (received.Count == 2)
                {
                    two.Set();
                }
            }
        };
        using Socket client = new(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
        client.Connect(IPAddress.Loopback, endpoint.Port);
        client.Send([0x10, 0x0C, 0, 4, (byte)'M', (byte)'Q', (byte)'T', (byte)'T', 4, 2, 0, 60, 0, 0]);
        ReadExactly(client, [0x20, 0x02, 0x00, 0x00]);
        // QoS 0, then QoS 1 with packet id 0x1234, sent in two pieces.
        byte[] qos0 = [0x30, 6, 0, 3, (byte)'a', (byte)'/', (byte)'b', 0x55];
        byte[] qos1 = [0x32, 9, 0, 3, (byte)'a', (byte)'/', (byte)'c', 0x12, 0x34, 1, 2];
        client.Send(qos0.Concat(qos1.Take(4)).ToArray());
        client.Send(qos1.Skip(4).ToArray());
        ReadExactly(client, [0x40, 0x02, 0x12, 0x34]);
        client.Send([0xC0, 0x00]);
        ReadExactly(client, [0xD0, 0x00]);
        two.Wait(2000).Should().BeTrue();
        received[0].Topic.Should().Be("a/b");
        received[0].Payload.Should().Equal(0x55);
        received[1].Topic.Should().Be("a/c");
        received[1].Payload.Should().Equal(1, 2);
        endpoint.Connects.Should().Be(1);
    }

    [Fact]
    public void A_replay_pauses_and_moves_to_another_time()
    {
        string path = Path.Combine(Path.GetTempPath(), "imustudio-test-" + Guid.NewGuid().ToString("N"));
        try
        {
            // 100 Hz, 10 samples a batch: a batch every 0.1 s, 2 s in all.
            using (RecordingWriter writer = new(path))
            {
                writer.Add("imu/rp2040/info", Encoding.UTF8.GetBytes(TestData.Info(odr: 100)));
                for (ulong i = 0; i < 20; i++)
                {
                    writer.Add("imu/rp2040/raw", TestData.Batch(i, i * 10, 10, (0, 0, 4098), (0, 0, 0), tick: i * 4000));
                }
                writer.Close(new Dictionary<string, object?>(), new Dictionary<string, byte[]>());
            }
            using RecordingPlayer player = new(path);
            player.Duration.Should().BeApproximately(2.0, 1e-9);
            List<string> played = new();
            int restarts = 0;
            using ManualResetEventSlim finished = new();
            player.Restarted += () => { lock (played) { restarts++; played.Add("restart"); } };
            player.Published += (topic, payload) =>
            {
                lock (played)
                {
                    played.Add(topic.EndsWith("/raw", StringComparison.Ordinal)
                        ? "raw" + ImuStudio.Core.Wire.ImuBatch.TryParse(payload, out _)!.Sequence
                        : "info");
                }
            };
            player.Finished += finished.Set;
            player.IsPaused = true;
            player.Start();
            // Paused from the start: the first payload shows where the replay is, then nothing.
            Thread.Sleep(300);
            lock (played) { played.Should().Equal("restart", "info", "raw0"); }
            player.Seek(1.75);
            Thread.Sleep(300);
            lock (played) { played.Should().Equal("restart", "info", "raw0", "restart", "info", "raw17"); }
            player.Position.Should().BeApproximately(1.8, 1e-9);
            player.IsPaused = false;
            finished.Wait(5000).Should().BeTrue();
            lock (played) { played.Skip(6).Should().Equal("raw18", "raw19"); }
            restarts.Should().Be(2);
        }
        finally
        {
            if (Directory.Exists(path))
            {
                Directory.Delete(path, true);
            }
        }
    }

    [Fact]
    public void A_recording_plays_back_what_was_written()
    {
        string path = Path.Combine(Path.GetTempPath(), "imustudio-test-" + Guid.NewGuid().ToString("N"));
        try
        {
            byte[] blob = TestData.CalibrationBlob(1, 70000, 0.001, (1, 2, 3));
            byte[][] batches =
            [
                TestData.Batch(0, 0, 8, (0, 0, 4098), (1, 2, 3), tick: 1000),
                TestData.Batch(1, 8, 8, (0, 0, 4098), (4, 5, 6), tick: 1048),
            ];
            using (RecordingWriter writer = new(path))
            {
                writer.Add("imu/rp2040/info", TestData.InfoBytes());
                writer.Add("imu/rp2040/raw", batches[0]);
                writer.Add("imu/rp2040/status",
                           Encoding.UTF8.GetBytes($$"""{"session":"{{TestData.Session}}","state":"measuring","tick":1040,"tick_mono_us":5}"""));
                writer.Add("imu/rp2040/raw", batches[1]);
                writer.Add("imu/rp2040/note", "not json"u8.ToArray());
                writer.Close(new Dictionary<string, object?> { ["batches"] = 2 },
                             new Dictionary<string, byte[]> { ["imu.cal.gyro.70000"] = blob });
            }
            File.ReadAllText(Path.Combine(path, "meta.json")).Should().Contain("\"batches\": 2");

            using RecordingPlayer player = new(path);
            player.Calibration["imu.cal.gyro.70000"].Should().Equal(blob);
            List<(string Topic, byte[] Payload)> played = new();
            using ManualResetEventSlim finished = new();
            player.Published += (topic, payload) => played.Add((topic, payload));
            player.Finished += finished.Set;
            player.Start();
            finished.Wait(5000).Should().BeTrue();
            played.Select(m => m.Topic.Split('/')[^1]).Should().Equal("info", "raw", "status", "raw");
            played[1].Payload.Should().Equal(batches[0]);
            played[3].Payload.Should().Equal(batches[1]);
            player.Duration.Should().BeApproximately(16.0 / 6667, 1e-9);
            player.Position.Should().Be(player.Duration);
        }
        finally
        {
            if (Directory.Exists(path))
            {
                Directory.Delete(path, true);
            }
        }
    }
}
