"""Short non-acceptance profiling run; never substitutes for settled cases."""
import cProfile
import pstats
import simulate

profile = cProfile.Profile()
profile.enable()
simulate.run('profile_only', dict(torque=False, brake=False, panto=False, settle_s=.02), duration=.02, replay=False)
profile.disable()
profile.dump_stats(str(simulate.ROOT/'runtime_profile.prof'))
pstats.Stats(profile).sort_stats('cumulative').print_stats(18)
