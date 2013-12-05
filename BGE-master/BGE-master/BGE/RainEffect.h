#pragma once
#include "ParticleEffect.h"
#include "Particle.h"
namespace BGE
{
	class RainEffect :
		public ParticleEffect
	{
	public:
		RainEffect(void);
		~RainEffect(void);

		bool Initialise();
		void InitParticle(Particle & particle);
		void UpdateParticle(float timeDelta, Particle & particle);
	};
}

