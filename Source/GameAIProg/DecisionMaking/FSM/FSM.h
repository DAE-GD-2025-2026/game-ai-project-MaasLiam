#pragma once

#include <any>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "CoreMinimal.h"

class AAIController;
class ASteeringAgent;
class UBlackboardComponent;

namespace GameAI::FSM
{
	class Blackboard
	{
	public:
		template<typename T>
		void Set(const std::string& Key, T Value)
		{
			Values[Key] = std::move(Value);
		}

		template<typename T>
		T* Get(const std::string& Key)
		{
			auto It = Values.find(Key);
			if (It == Values.end()) return nullptr;
			return std::any_cast<T>(&It->second);
		}

		template<typename T>
		T GetOr(const std::string& Key, const T& DefaultValue) const
		{
			auto It = Values.find(Key);
			if (It == Values.end()) return DefaultValue;
			if (const T* Value = std::any_cast<T>(&It->second)) return *Value;
			return DefaultValue;
		}

		bool Contains(const std::string& Key) const
		{
			return Values.contains(Key);
		}

	private:
		std::unordered_map<std::string, std::any> Values{};
	};

	struct Context
	{
		AAIController* Controller{nullptr};
		ASteeringAgent* Agent{nullptr};
		UBlackboardComponent* UnrealBlackboard{nullptr};
		Blackboard* Blackboard{nullptr};
	};

	class State
	{
	public:
		virtual ~State() = default;

		virtual void OnEnter(Context& Ctx) {}
		virtual void Tick(Context& Ctx, float DeltaTime) {}
		virtual void OnExit(Context& Ctx) {}
	};

	class Transition
	{
	public:
		Transition(State* InFrom, State* InTo, std::function<bool()> InCondition)
			: From(InFrom), To(InTo), Condition(std::move(InCondition)) {}

		State* From{nullptr};
		State* To{nullptr};
		std::function<bool()> Condition{};
	};

	class FSM
	{
	public:
		explicit FSM(Context InContext = {});

		void SetContext(Context InContext);
		Context& GetContext() { return Ctx; }
		Blackboard& GetBlackboard() { return BlackboardInstance; }

		State* AddState(std::unique_ptr<State>&& NewState);
		void AddTransition(State* From, State* To, std::function<bool()> Condition);

		void Start();
		void Stop();
		void Tick(float DeltaTime);

		State* GetCurrentState() const { return CurrentState; }

	private:
		void ChangeState(State* NewState);

		std::vector<std::unique_ptr<State>> States{};
		std::vector<Transition> Transitions{};
		State* CurrentState{nullptr};
		Context Ctx{};
		Blackboard BlackboardInstance{};
		bool bStarted{false};
	};
}
