#include "FSM.h"

namespace GameAI::FSM
{
	FSM::FSM(Context InContext)
	{
		SetContext(InContext);
	}

	void FSM::SetContext(Context InContext)
	{
		Ctx = InContext;
		Ctx.Blackboard = &BlackboardInstance;
	}

	State* FSM::AddState(std::unique_ptr<State>&& NewState)
	{
		State* RawState = NewState.get();
		States.emplace_back(std::move(NewState));

		if (!CurrentState)
		{
			CurrentState = RawState;
		}

		return RawState;
	}

	void FSM::AddTransition(State* From, State* To, std::function<bool()> Condition)
	{
		Transitions.emplace_back(From, To, std::move(Condition));
	}

	void FSM::Start()
	{
		if (bStarted || !CurrentState) return;
		bStarted = true;
		CurrentState->OnEnter(Ctx);
	}

	void FSM::Stop()
	{
		if (!bStarted) return;
		if (CurrentState) CurrentState->OnExit(Ctx);
		bStarted = false;
	}

	void FSM::Tick(float DeltaTime)
	{
		if (!bStarted || !CurrentState) return;

		for (const Transition& T : Transitions)
		{
			if (T.From == CurrentState && T.To && T.Condition && T.Condition())
			{
				ChangeState(T.To);
				break;
			}
		}

		if (CurrentState)
		{
			CurrentState->Tick(Ctx, DeltaTime);
		}
	}

	void FSM::ChangeState(State* NewState)
	{
		if (!NewState || NewState == CurrentState) return;
		if (CurrentState) CurrentState->OnExit(Ctx);
		CurrentState = NewState;
		CurrentState->OnEnter(Ctx);
	}
}
