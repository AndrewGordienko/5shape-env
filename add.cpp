#include <iostream>
#include <box2d/box2d.h>
#include <SDL2/SDL.h>
#include <random>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <vector>

class Floor {
public:
    Floor(b2World& world, float y) {
        b2BodyDef groundBodyDef;
        groundBodyDef.position.Set(20.0f, y);  // Adjusted position
        groundBody = world.CreateBody(&groundBodyDef);

        b2PolygonShape groundBox;
        groundBox.SetAsBox(20.0f, 1.0f);  

        b2FixtureDef groundFixtureDef;
        groundFixtureDef.shape = &groundBox;
        groundBody->CreateFixture(&groundFixtureDef);
    }

    b2Body* GetGroundBody() {
        return groundBody;
    }

private:
    b2Body* groundBody;
};

class Shape {
public:
    Shape(b2World& world) {
        const float shapeX = 20.0f;  
        const float shapeY = 10.0f;  // Adjusted position

        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.position.Set(shapeX, shapeY);
        box = world.CreateBody(&bodyDef);

        int numSides = std::rand() % 5 + 3; // Random number of sides between 4 and 8
        float distance = 1.0f * std::sin(b2_pi / numSides);  // Increased size
        vertices.reserve(numSides);
        for (int i = 0; i < numSides; i++) {
            float x = distance * std::cos(2 * b2_pi / numSides * i);
            float y = distance * std::sin(2 * b2_pi / numSides * i);
            vertices.emplace_back(x, y);
        }

        b2PolygonShape boxShape;
        boxShape.Set(vertices.data(), numSides);
        boxFixture.shape = &boxShape;
        boxFixture.density = 1.0f;
        boxFixture.friction = 0.1f;
        box->CreateFixture(&boxFixture);
    }

    b2Body* GetBoxBody() const {
        return box;
    }

    const b2Vec2* GetVertices() const {
        return vertices.data();
    }

    int GetNumVertices() const {
        return vertices.size();
    }

private:
    b2Body* box;
    b2FixtureDef boxFixture;
    std::vector<b2Vec2> vertices;
};


class Joint {
public:
    Joint(b2World& world, Shape& shape1, Shape& shape2) {
        b2RevoluteJointDef jointDef;
        
        // Randomly select vertices from the two shapes
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, shape1.GetNumVertices() - 1);
        int index1 = dis(gen);
        int index2 = dis(gen);

        jointDef.bodyA = shape1.GetBoxBody();
        jointDef.bodyB = shape2.GetBoxBody();
        jointDef.localAnchorA = shape1.GetVertices()[index1];
        jointDef.localAnchorB = shape2.GetVertices()[index2];

        jointDef.enableMotor = true;
        jointDef.motorSpeed = 50; 
        jointDef.maxMotorTorque = 750; 

        joint = static_cast<b2RevoluteJoint*>(world.CreateJoint(&jointDef));
    }

    b2RevoluteJoint* GetJoint() {
        return joint;
    }

    float GetMotorSpeed() const {
        return joint->GetMotorSpeed();
    }

private:
    b2RevoluteJoint* joint;
};

class Creature {
public:
    Creature(b2World& world, int numShapes = 5, const std::vector<int>& chainLengths = {3, 2}) {
        float distanceBetweenShapes = 20.0f;  // Distance between shapes
        float initialPositionY = 10.0f;  // Initial vertical position

        // Create the shapes
        for(int i = 0; i < numShapes; ++i) {
            Shape shape(world);
            shape.GetBoxBody()->SetTransform(b2Vec2(20.0f, initialPositionY + i * distanceBetweenShapes), 0);  // Set position
            shapes.push_back(std::move(shape));
        }

        std::vector<std::pair<Shape*, Shape*>> connections;

        // Connect shapes within each chain
        for(int i = 0; i < chainLengths.size(); ++i) {
            int chainStart = std::accumulate(chainLengths.begin(), chainLengths.begin() + i, 0);
            for(int j = 0; j < chainLengths[i] - 1; ++j) {
                connections.emplace_back(&shapes[chainStart + j], &shapes[chainStart + j + 1]);
            }
        }

        // Connect chains together
        for(int i = 0; i < chainLengths.size() - 1; ++i) {
            int chain1Start = std::accumulate(chainLengths.begin(), chainLengths.begin() + i, 0);
            int chain2Start = std::accumulate(chainLengths.begin(), chainLengths.begin() + i + 1, 0);
            connections.emplace_back(&shapes[chain1Start], &shapes[chain2Start]);
        }

        // Randomly shuffle the connections
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(connections.begin(), connections.end(), g);

        // Create the joints
        for(auto& connection : connections) {
            joints.emplace_back(world, *connection.first, *connection.second);
        }
    }

    const std::vector<Shape>& getShapes() const {
        return shapes;
    }

private:
    std::vector<Shape> shapes;
    std::vector<Joint> joints;
};

class RayCastCallback : public b2RayCastCallback {
public:
    RayCastCallback()
        : hit(false), distance(-1.0f) {}

    float ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float fraction) override {
        hit = true;
        distance = fraction; 
        return 0.0f; // Returning 0 terminates the raycast after the first hit
    }

    bool hit;
    float distance;
};

void checkCollisionWithGroundAndRayCast(Creature& creature, b2World& world, Floor& floor, std::vector<float>& touch_inputs, std::vector<float>& raycast_result, std::vector<int>& directions) {
    for (int index = 0; index < creature.getShapes().size(); ++index) {
        const Shape& shape = creature.getShapes()[index];
        for (b2ContactEdge* edge = shape.GetBoxBody()->GetContactList(); edge; edge = edge->next) {
            if (edge->other == floor.GetGroundBody()) {
                touch_inputs[index-1] = 1;
                break;
            }
        }

        if (index == 3) {  // For the middle shape (shape number three)
            b2Vec2 center = shape.GetBoxBody()->GetPosition();  // Center point of the shape
            float raycast_length = 100.0f;  // Length of the rays

            RayCastCallback callback;  // Create an instance of the callback object

            for (int direction : directions) {
                float angle = static_cast<float>(direction) * (b2_pi / 4.0f);  // Convert the direction to radians
                b2Vec2 raycast_end = center + raycast_length * b2Vec2(std::cos(angle), std::sin(angle));

                world.RayCast(&callback, center, raycast_end);  // Perform the raycast for each direction
                if (callback.hit) {
                    raycast_result[direction] = raycast_length * callback.distance;
                }

                callback.hit = false;  // Reset the hit flag for the next raycast
            }
        }
    }
}

b2Vec2 box2dToScreen(const b2Vec2& input, int windowHeight) {
    float scale = 20.0f; 
    b2Vec2 output;
    output.x = scale * input.x;
    output.y = windowHeight - scale * input.y;
    return output;
}

void drawBox() {
    std::srand(std::time(nullptr));

    const int SCREEN_WIDTH = 800;
    const int SCREEN_HEIGHT = 800;

    b2Vec2 gravity(0.0f, -9.8f);  // Increased gravity
    b2World world(gravity);

    Floor floor(world, 1.0f); // Halfway down the screen
    Creature creature(world); // Create the creature with default parameters

    SDL_Init(SDL_INIT_VIDEO);

    SDL_Window* window = SDL_CreateWindow("Box2D Physics Simulation", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    // Initialize directions for the raycast (45 degrees apart)
    std::vector<int> directions = {0, 1, 2, 3, 4, 5, 6, 7};

    // Initialize touch_inputs and raycast_result with zeroes
    std::vector<float> touch_inputs(creature.getShapes().size(), 0.0f);
    std::vector<float> raycast_result(directions.size(), 0.0f);

    bool running = true;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
        }

        float timeStep = 1.0f / 60.0f; 
        int32 velocityIterations = 6;
        int32 positionIterations = 2;
        
        world.Step(timeStep, velocityIterations, positionIterations);

        checkCollisionWithGroundAndRayCast(creature, world, floor, touch_inputs, raycast_result, directions);

        // Append touch_inputs to the end of raycast_result
        raycast_result.insert(raycast_result.end(), touch_inputs.begin(), touch_inputs.end());

        // Print out the values of raycast_result
        std::cout << "Raycast result and collision:";
        for (const auto& value : raycast_result) {
            std::cout << " " << value;
        }
        std::cout << std::endl;

        // Remove the appended touch_inputs from raycast_result for the next iteration
        raycast_result.resize(directions.size());

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        SDL_SetRenderDrawColor(renderer, 128, 128, 128, 255);
        SDL_Rect groundRect;
        b2Vec2 floorPosition = box2dToScreen(floor.GetGroundBody()->GetPosition(), SCREEN_HEIGHT);
        groundRect.x = static_cast<int>(floorPosition.x - SCREEN_WIDTH / 2.0f);
        groundRect.y = static_cast<int>(floorPosition.y - 10.0f);
        groundRect.w = SCREEN_WIDTH;
        groundRect.h = 20;
        SDL_RenderFillRect(renderer, &groundRect);

        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

        for (const auto& shape : creature.getShapes()) {
            b2Vec2 shapePosition = box2dToScreen(shape.GetBoxBody()->GetPosition(), SCREEN_HEIGHT);
            const b2Vec2* vertices = shape.GetVertices();
            int numVertices = shape.GetNumVertices();

            for (int i = 0; i < numVertices; ++i) {
                int nextIndex = (i + 1) % numVertices;
                b2Vec2 vertex1 = box2dToScreen(vertices[i] + shape.GetBoxBody()->GetPosition(), SCREEN_HEIGHT);
                b2Vec2 vertex2 = box2dToScreen(vertices[nextIndex] + shape.GetBoxBody()->GetPosition(), SCREEN_HEIGHT);

                SDL_RenderDrawLine(renderer,
                    static_cast<int>(vertex1.x),
                    static_cast<int>(vertex1.y),
                    static_cast<int>(vertex2.x),
                    static_cast<int>(vertex2.y));
            }
        }

        SDL_RenderPresent(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}
