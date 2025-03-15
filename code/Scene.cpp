//
//  Scene.cpp
//
#include "Scene.h"
#include "../Shape.h"
#include "../Intersections.h"


/*
========================================================================================================

Scene

========================================================================================================
*/

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene() {
	for ( int i = 0; i < bodies.size(); i++ ) {
		delete bodies[ i ]->shape;
		delete bodies[i];
	}
	bodies.clear();
}



/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() {
	for ( int i = 0; i < bodies.size(); i++ ) {
		delete bodies[ i ]->shape;
		delete bodies[i];
	}
	bodies.clear();
	
	//Reset Gameplay
	p1CurrentBall = -1;
	p2CurrentBall = 0;

	Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize() {
	int mapSize = 10;
	int min = -floor(mapSize / 2);
	int max = -min;
	float ballOffset = 20;
	//Map Creation;

	//Cochonnet Spawn
	Cochonnet = new Body();
	Cochonnet->position = Vec3(3, -2.2, 0.3f);
	Cochonnet->orientation = Quat(0, 0, 0, 1);
	Cochonnet->shape = new ShapeSphere(0.4f);
	Cochonnet->inverseMass = 100.0f;
	Cochonnet->elasticity = 0.1f;
	Cochonnet->friction = 1.0f;
	Cochonnet->linearVelocity = Vec3(0, 0, 0);
	bodies.push_back(Cochonnet);

	for (int i = 0; i < (MaxPetaqueBall*2); i++) {
		PetanqueBall* ball = new PetanqueBall();
		Body* b = new Body();

		//Spawn under
		b->position = Vec3(0, 0, -100000.0f);
		b->orientation = Quat(0, 0, 0, 1);
		b->shape = new ShapeSphere(1.0f);
		//Dissable Physics
		b->inverseMass = 0.0f;
		b->elasticity = 0.1f;
		b->friction = 0.5f;
		b->linearVelocity = Vec3(0, 0, 0);
		ball->body = b;
		ball->isPlayer1 = !isPlayer1Winning;

		PlayerBall[i] = ball;
		bodies.push_back(ball->body);
	}
	

	for (int x = min; x <= max; x++) {
		for (int y = min; y <= max; y++) {
			//Wall
			if (x == min || x == max || y == min || y == max) {
				Body* wall = new Body();
				wall->position = Vec3(x * ballOffset, y * ballOffset, ballOffset);
				wall->orientation = Quat(0, 0, 0, 1);
				wall->shape = new ShapeSphere(ballOffset*2);
				wall->inverseMass = 0.0f;
				wall->elasticity = 0.99f;
				wall->friction = 0.5f;
				bodies.push_back(wall);
			}
			else {
				Body* earth = new Body();
				earth->position = Vec3(x*ballOffset, y*ballOffset, -1000);
				earth->orientation = Quat(0, 0, 0, 1);
				earth->shape = new ShapeSphere(1000.0f);
				earth->inverseMass = 0.0f;
				earth->elasticity = 0.99f;
				earth->friction = 0.5f;
				bodies.push_back(earth);
			}
		}
	}
	
	setNewPhase(gamePhase::Launching);

}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update(const float dt_sec)
{
	//Gameplay Loop 
	CameraCenter = getCurrentBall()->body->position;

	if (currentPhase == gamePhase::Waiting) {
		if (std::chrono::system_clock::now() > pauseTill) {
			for (int i = 0; i <= (p1CurrentBall + p2CurrentBall); i++) {
				PetanqueBall* currBall = PlayerBall[i];
				currBall->body->linearVelocity.Zero();
				currBall->body->angularVelocity.Zero();
			}
			Cochonnet->linearVelocity.Zero();
			Cochonnet->angularVelocity.Zero();
			//Show Results
			setNewPhase(gamePhase::Result);
			//GO back to the game
			setNewPhase(gamePhase::Launching);
		}
	}


	// Gravity
	
	for (int i = 0; i < bodies.size(); ++i)
	{
		Body* body = bodies[i];
		float mass = 1.0f / body->inverseMass;
		// Gravity needs to be an impulse I
		// I == dp, so F == dp/dt <=> dp = F * dt
		// <=> I = F * dt <=> I = m * g * dt
		Vec3 impulseGravity = Vec3(0, 0, -10) * mass * dt_sec;
		body->ApplyImpulseLinear(impulseGravity);
	}

	
	

	// Collision checks
	
	int numContacts = 0;
	const int maxContacts = bodies.size() * bodies.size();
	Contact* contacts = (Contact*)_malloca(sizeof(Contact) * maxContacts);
	for (int i = 0; i < bodies.size(); ++i)
	{
		for (int j = i + 1; j < bodies.size(); ++j)
		{
			Body* bodyA = bodies[i];
			Body* bodyB = bodies[j];
			if (bodyA->inverseMass == 0.0f && bodyB->inverseMass == 0.0f)
				continue;
			Contact contact;
			if (Intersections::Intersect(bodyA, bodyB, dt_sec, contact))
			{
				contacts[numContacts] = contact;
				++numContacts;
			}
		}
	}
	// Sort times of impact
	if (numContacts > 1) {
		qsort(contacts, numContacts, sizeof(Contact),
			Contact::CompareContact);
	}
	// Contact resolve in order
	
	float accumulatedTime = 0.0f;
	for (int i = 0; i < numContacts; ++i)
	{
		Contact& contact = contacts[i];
		const float dt = contact.timeOfImpact - accumulatedTime;
		Body* bodyA = contact.a;
		Body* bodyB = contact.b;
		// Skip body par with infinite mass
		if (bodyA->inverseMass == 0.0f && bodyB->inverseMass == 0.0f)
			continue;
		// Position update
		for (int j = 0; j < bodies.size(); ++j) {
			bodies[j]->Update(dt);
		}
		Contact::ResolveContact(contact);
		accumulatedTime += dt;
	}
	// Other physics behavirous, outside collisions.
	// Update the positions for the rest of this frame's time.
	const float timeRemaining = dt_sec - accumulatedTime;
	if (timeRemaining > 0.0f)
	{
		for (int i = 0; i < bodies.size(); ++i) {
			bodies[i]->Update(timeRemaining);
		}
	}
	
	
}

/*
====================================================
Scene::Gameplay
====================================================
*/


void Scene::launchCurrentBall(Vec3 dir)
{
	if (currentPhase == gamePhase::Launching) {
		getCurrentBall()->body->inverseMass = 3.0f;
		getCurrentBall()->body->ApplyImpulseLinear(dir * 5);
		setNewPhase(gamePhase::Waiting);
		pauseTill = std::chrono::system_clock::now() + std::chrono::seconds(7);
	}
}

void Scene::setNewPhase(gamePhase newPhase)
{
	currentPhase = newPhase;
	std::stringstream ss;
	float minDist = 999999;
	switch (newPhase) {
	case gamePhase::Launching:

		
		//check if a player has already thrown enough ball
		if ((p2CurrentBall == MaxPetaqueBall && isPlayer1Winning) || (p1CurrentBall == MaxPetaqueBall && !isPlayer1Winning)) {
			setNewPhase(gamePhase::FinalResult);
			break;
		}

		if (isPlayer1Winning) {
			p2CurrentBall++;
		}
		else {
			p1CurrentBall++;
		}


		ss << " Waiting for ";
		ss << (isPlayer1Winning ? "Player 2 " : "Player 1 ") ;
		ss << "To launch the petanque ball !" << std::endl;
		std::cout << ss.str();

		createBall();

		
		break;
	case gamePhase::Waiting:
		ss << " Waiting for ";
		ss << (isPlayer1Winning ? "Player 2 " : "Player 1 ");
		ss << "ball to stop !" << std::endl;
		std::cout << ss.str();
		break;
	case gamePhase::Result:

		for (int i = 0; i <= (p1CurrentBall + p2CurrentBall); i++) {
			PetanqueBall* currBall = PlayerBall[i];
			float dist = (currBall->body->position - Cochonnet->position).GetMagnitude();
			if (minDist > dist ) {
				minDist = dist;
				isPlayer1Winning = currBall->isPlayer1;
			}
		}
		
		ss << (isPlayer1Winning ? "Player 1 " : "Player 2 ");
		ss << "is winning !" << std::endl;
		std::cout << ss.str();
		break;
	case gamePhase::FinalResult:

		for (int i = 0; i <= (p1CurrentBall + p2CurrentBall); i++) {
			PetanqueBall* currBall = PlayerBall[i];
			float dist = (currBall->body->position - Cochonnet->position).GetMagnitude();
			if (minDist > dist) {
				minDist = dist;
				isPlayer1Winning = currBall->isPlayer1;
			}
		}
		ss << (isPlayer1Winning ? "Player 1 " : "Player 2 ");
		ss << "won the game !" << std::endl;
		std::cout << ss.str();
		break;
	}

}

void Scene::createBall()
{
	PetanqueBall* pb = getCurrentBall();
	//Spawn under
	pb->body->position = Vec3(0, 0, 1.2f);
	pb->isPlayer1 = !isPlayer1Winning;
}

PetanqueBall* Scene::getCurrentBall() {
	return PlayerBall[p1CurrentBall + p2CurrentBall];
}
