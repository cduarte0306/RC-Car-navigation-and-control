#include "Modules_Lib/AdapterBase.hpp"
#include "lib/RegisterMap.hpp"
#include "utils/logger.hpp"

namespace Adapter {

AdapterBase::AdapterBase(ModuleDefs::AdapterId id, std::string parentName_) : 
parentName(parentName_), 
adapterId(id),
m_ReplyMailBox(100),
m_InputMailBox(100),
m_ReplyThreadRunning(true)
{
	// Resolve the parent module's ID
	RegisterMap* regMap = RegisterMap::getInstance();
	if (regMap)
	{
		if (auto moduleMap = regMap->get<std::unordered_map<std::string, int>>(RegisterMap::RegisterKeys::ModuleMap))
		{
			auto it = moduleMap->find(parentName_);
			if (it != moduleMap->end())
			{
				m_ModuleID = it->second;
			}
		}
	}

	// Start the reply listening thread
	m_ReplyProcThread = std::thread(&AdapterBase::procReplyThread, this);
	m_InputProcThread = std::thread(&AdapterBase::procInputThread, this);
}

AdapterBase::~AdapterBase()
{
	m_ReplyThreadRunning = false;
	m_ReplyMailBox.flush();
	m_InputMailBox.flush();

	if (m_ReplyProcThread.joinable())
	{
		m_ReplyProcThread.join();
	}
	if (m_InputProcThread.joinable())
	{
		m_InputProcThread.join();
	}
}

void AdapterBase::procReplyThread(void)
{
	int ret = 0;
	while (m_ReplyThreadRunning)
	{
		Msg::MessageAck<std::vector<char>>& ack = m_ReplyMailBox.getHead();
		Logger::getLoggerInst()->log(Logger::LOG_LVL_DEBUG, "Adapter %s processing reply for Command ID: %d, Seq ID: %d, Adapter parent module ID: %d\r\n",
			parentName.c_str(), ack.mCommandID, ack.mSeqID, m_ModuleID);

		if (replyHandlerFunc)
		{
			ret = replyHandlerFunc(ack);
			if (ret < 0)
			{
				Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Error processing reply in adapter\r\n");
			}
		}
		m_ReplyMailBox.pop();
	}
}

void AdapterBase::procInputThread(void)
{
	int ret = 0;
	while (true)
	{
		auto& capsule = m_InputMailBox.getHead();
		if (moduleDispatchCmd)
		{
			ret = moduleDispatchCmd(capsule);
			if (ret < 0)
			{
				Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Error dispatching input capsule in adapter\r\n");
			}
		}
		else if (OnModuleMsgReceivedFunc)
		{
			auto& buffer = capsule.getData();
			ret = OnModuleMsgReceivedFunc(buffer);
			if (ret < 0)
			{
				Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Error processing input message in adapter\r\n");
			}
		}

		m_InputMailBox.pop();
	}
}

int AdapterBase::SetModDispatchCallback(std::function<int(Msg::MessageCapsule<std::vector<char>>&)> modDispatchCB)
{
	if (modDispatchCB == nullptr) return -1;
	moduleDispatchCmd = modDispatchCB;
	return 0;
}

int AdapterBase::SetReplyHandlerCallback(std::function<int(Msg::MessageAck<std::vector<char>>&)> replyHandlerCB)
{
	if (replyHandlerCB == nullptr) return -1;
	replyHandlerFunc = replyHandlerCB;
	return 0;
}

int AdapterBase::bind(AdapterBase* Adapter)
{
	adapterMap[Adapter->getParentName()] = Adapter;

	this->moduleWriteCmd = [Adapter](char* pbuf, size_t len)
	{
		return Adapter->moduleCommand_(pbuf, len);
	};

	this->moduleWriteCmdVector = [Adapter](std::vector<char>& buffer)
	{
		return Adapter->moduleCommand_(buffer);
	};

	this->dispatchCommandFunc = [Adapter](Msg::MessageCapsule<std::vector<char>>& capsule)
	{
		return Adapter->SubmitMailBox(capsule);
	};
	
	// Route replies from this source adapter to the bound destination adapter.
	this->moduleReplyCmd = [Adapter](Msg::MessageAck<std::vector<char>>& ack)
	{
		return Adapter->SubmitReplyMailBox(ack);
	};

	return bind_(Adapter);
}

int AdapterBase::bindCommandDispatch(std::function<int(Msg::MessageCapsule<std::vector<char>>& capsule)> dispatchFunc)
{
	if (!dispatchFunc)
	{
		return -1;
	}

	this->dispatchCommandFunc = dispatchFunc;
	return 0;
}

int AdapterBase::bindOnModuleMsgReceived(std::function<int(std::vector<char>& buffer)> onMsgReceivedFunc)
{
	if (!onMsgReceivedFunc)
	{
		return -1;
	}

	this->OnModuleMsgReceivedFunc = onMsgReceivedFunc;
	return 0;
}

std::string AdapterBase::readStats()
{
	if (!readStatsCommand)
	{
		return "";
	}

	return readStatsCommand();
}

int AdapterBase::stopCmd(void)
{
	return 0;
}

int AdapterBase::startCmd(void)
{
	return 0;
}

int AdapterBase::dispatchCommand(Msg::MessageCapsule<std::vector<char>>& capsule)
{
	if (!dispatchCommandFunc)
	{
		return -1;
	}

	dispatchCommandFunc(capsule);
	return 0;
}

int AdapterBase::moduleCommand(char* pbuf, size_t len)
{
	if (!moduleWriteCmd)
	{
		std::cout << "Module command error\r\n";
		return -1;
	}

	moduleWriteCmd(pbuf, len);
	return 0;
}

int AdapterBase::moduleCommand(std::vector<char>& buffer)
{
	if (!moduleWriteCmd)
	{
		std::cout << "Module command error\r\n";
		return -1;
	}

	return moduleWriteCmdVector(buffer);
}

int AdapterBase::cliCommand(std::vector<std::string>& buffer)
{
	if (!moduleCliCmd)
	{
		std::cout << "Module CLI command error\r\n";
		return -1;
	}

	return moduleCliCmd(buffer);
}

std::string AdapterBase::getParentName() const
{
	return parentName;
}

int AdapterBase::AckMsg(Msg::MessageCapsule<std::vector<char>>& capsule, char* reply, int len)
{
	if (!reply || len <= 0)
	{
		return -1;
	}

	int ret = capsule.SendAck(capsule.getSeqID(), reply, len);
	if (ret != 0)
	{
		return ret;
	}

	Msg::MessageAck<std::vector<char>> ack(true, capsule.GetAckRaw());
	ack.mCommandID = capsule.getCommand();
	ack.mSeqID = capsule.getSeqID();
	ack.mReplyDestID = capsule.getSource();
	return ConnectModuleReply(ack);
}

int AdapterBase::ConnectModuleReply(Msg::MessageAck<std::vector<char>>& ack)
{
	if (moduleReplyCmd)
	{
		return moduleReplyCmd(ack);
	}

	return SubmitReplyMailBox(ack);
}

int AdapterBase::moduleCommand_(char* pbuf, size_t len)
{
	(void)pbuf;
	(void)len;
	return -1;
}

int AdapterBase::moduleCommand_(std::vector<char>& buffer)
{
	(void)buffer;
	return -1;
}

int AdapterBase::moduleCliCmd_(std::vector<std::string>& buffer)
{
	(void)buffer;
	return -1;
}

} // namespace Adapter
