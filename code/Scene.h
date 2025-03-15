//
//  Scene.h
//
#pragma once
#include <vector>

#include "../Body.h"
#include <iostream>
#include <sstream>
#include <chrono>

const int MaxPetaqueBall = 3;
struct PetanqueBall {
	Body* body;
	bool isPlayer1 = false;
};

enum gamePhase {
	Launching,
	Waiting,
	Result,
	FinalResult
};

/*
====================================================
Scene
====================================================
*/
class Scene {
public:
	Scene() { bodies.reserve( 128 ); }
	~Scene();

	void Reset();
	void Initialize();
	void Update( const float dt_sec );	

	std::vector<Body*> bodies;

	//Game Related
	
	std::chrono::system_clock::time_point pauseTill;

	Vec3 CameraCenter;

	Body* Cochonnet;

	gamePhase currentPhase = gamePhase::Launching;

	void launchCurrentBall(Vec3 dir);
	void setNewPhase(gamePhase newPhase);
	void createBall();
	PetanqueBall* getCurrentBall();
	
	

	bool isPlayer1Winning = false;

	PetanqueBall* PlayerBall[MaxPetaqueBall*2];
	int p1CurrentBall = -1;
	int p2CurrentBall = 0;
};



