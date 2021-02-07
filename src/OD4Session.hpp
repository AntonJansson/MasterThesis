/*
  * Copyright (C) 2017-2018  Christian Berger
  *
  * This Source Code Form is subject to the terms of the Mozilla Public
  * License, v. 2.0. If a copy of the MPL was not distributed with this
  * file, You can obtain one at http://mozilla.org/MPL/2.0/.
  */
 
 #ifndef CLUON_OD4SESSION_HPP
 #define CLUON_OD4SESSION_HPP
 
 #include "cluon/Time.hpp"
 #include "cluon/ToProtoVisitor.hpp"
 #include "cluon/UDPReceiver.hpp"
 #include "cluon/UDPSender.hpp"
 #include "cluon/cluon.hpp"
 #include "cluon/cluonDataStructures.hpp"
 
 #include <chrono>
 #include <cstdint>
 #include <functional>
 #include <memory>
 #include <mutex>
 #include <string>
 #include <unordered_map>
 #include <utility>
 
 namespace cluon {
 class LIBCLUON_API OD4Session {
    private:
     OD4Session(const OD4Session &) = delete;
     OD4Session(OD4Session &&)      = delete;
     OD4Session &operator=(const OD4Session &) = delete;
     OD4Session &operator=(OD4Session &&) = delete;
 
    public:
     OD4Session(uint16_t CID, std::function<void(cluon::data::Envelope &&envelope)> delegate = nullptr) noexcept;
 
     void send(cluon::data::Envelope &&envelope) noexcept;
 
     bool dataTrigger(int32_t messageIdentifier, std::function<void(cluon::data::Envelope &&envelope)> delegate) noexcept;
 
     void timeTrigger(float freq, std::function<bool()> delegate) noexcept;
 
     template <typename T>
     void send(T &message, const cluon::data::TimeStamp &sampleTimeStamp = cluon::data::TimeStamp(), uint32_t senderStamp = 0) noexcept {
         try {
             std::lock_guard<std::mutex> lck(m_senderMutex);
             cluon::ToProtoVisitor protoEncoder;
 
             cluon::data::Envelope envelope;
             {
                 envelope.dataType(static_cast<int32_t>(message.ID()));
                 message.accept(protoEncoder);
                 envelope.serializedData(protoEncoder.encodedData());
                 envelope.sent(cluon::time::now());
                 envelope.sampleTimeStamp((0 == (sampleTimeStamp.seconds() + sampleTimeStamp.microseconds())) ? envelope.sent() : sampleTimeStamp);
                 envelope.senderStamp(senderStamp);
             }
 
             send(std::move(envelope));
         } catch (...) {} // LCOV_EXCL_LINE
     }
 
    public:
     bool isRunning() noexcept;
 
    private:
     void callback(std::string &&data, std::string &&from, std::chrono::system_clock::time_point &&timepoint) noexcept;
     void sendInternal(std::string &&dataToSend) noexcept;
 
    private:
     std::unique_ptr<cluon::UDPReceiver> m_receiver;
     cluon::UDPSender m_sender;
 
     std::mutex m_senderMutex{};
 
     std::function<void(cluon::data::Envelope &&envelope)> m_delegate{nullptr};
 
     std::mutex m_mapOfDataTriggeredDelegatesMutex{};
     std::unordered_map<int32_t, std::function<void(cluon::data::Envelope &&envelope)>, UseUInt32ValueAsHashKey> m_mapOfDataTriggeredDelegates{};
 };
 
 } // namespace cluon
 #endif
