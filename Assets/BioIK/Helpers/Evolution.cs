using System.Collections.Generic;
using UnityEngine;

namespace BioIK.Helpers {

	//----------------------------------------------------------------------------------------------------
	//====================================================================================================
	//Memetic Evolutionary Optimisation
	//====================================================================================================
	//----------------------------------------------------------------------------------------------------
	public class Evolution {		
		private readonly Model _model;          //Reference to the kinematic model
		private readonly int _populationSize;   //Number of individuals (population size)
		private readonly int _elites;           //Number of elite individuals
		private readonly int _dimensionality;   //Search space dimensionality

		private readonly double[] _lowerBounds; //Constraints for the lower bounds
		private readonly double[] _upperBounds; //Constraints for the upper bounds

		private Individual[] _population;   //Array for current individuals
		private Individual[] _offspring;    //Array for offspring individuals

		private readonly List<Individual> _pool = new();    //Selection pool for recombination
		private int _poolCount;                             //Current size of the selection pool
		private readonly double[] _probabilities;           //Current probabilities for selection
		private double _gene;                               //Simple storage variable #1
		private double _weight;                             //Simple storage variable #2

        //Variables for elitism exploitation
        private bool _evolving;
        private readonly bool[] _improved;
        private readonly Model[] _models;
        private readonly Bfgs[] _optimisers;

        //Variables for optimisation queries
		private readonly double[] _solution;    //Evolutionary solution
		private double _fitness;                //Evolutionary fitness

		//Initialises the algorithm
		public Evolution(Model model, int populationSize, int elites)
        {
			_model = model;
			_populationSize = populationSize;
			_elites = elites;
			_dimensionality = model.GetDoF();

			_population = new Individual[_populationSize];
			_offspring = new Individual[_populationSize];
			for (int i = 0; i < _populationSize; i++)
            {
				_population[i] = new(_dimensionality);
				_offspring[i] = new(_dimensionality);
			}

			_lowerBounds = new double[_dimensionality];
			_upperBounds = new double[_dimensionality];
			_probabilities = new double[_populationSize];
			_solution = new double[_dimensionality];

            _models = new Model[_elites];
            _optimisers = new Bfgs[_elites];
            _improved = new bool[_elites];
            for (int i = 0; i < _elites; i++)
            {
                int index = i;
                _models[index] = new(_model.GetCharacter());
                _optimisers[index] = new(_dimensionality, x => _models[index].ComputeLoss(x), y => _models[index].ComputeGradient(y, 1e-5));
            }
        }

		public double[] Optimise(int generations, double[] seed)
        {
            _model.Refresh();
            
			for (int i = 0; i < _dimensionality; i++)
            {
				_lowerBounds[i] = _model.MotionPointers[i].Motion.GetLowerLimit(true);
				_upperBounds[i] = _model.MotionPointers[i].Motion.GetUpperLimit(true);
				_solution[i] = seed[i];
			}
			_fitness = _model.ComputeLoss(_solution);

            Initialise(seed);
            for (int i = 0; i < _elites; i++)
            {
                _models[i].CopyFrom(_model);
                _optimisers[i].LowerBounds = _lowerBounds;
                _optimisers[i].UpperBounds = _upperBounds;
            }
            for (int i = 0; i < generations; i++)
            {
                Evolve();
            }

			return _solution;
		}

		//Initialises the population with the seed
		private void Initialise(double[] seed)
        {
			for (int i = 0; i < _dimensionality; i++)
            {
				_population[0].Genes[i] = seed[i];
				_population[0].Momentum[i] = 0.0;
			}
			_population[0].Fitness = _model.ComputeLoss(_population[0].Genes);

			for (int i=1; i < _populationSize; i++)
            {
				Reroll(_population[i]);
			}

			SortByFitness();

            ComputeExtinctions();

			TryUpdateSolution();
		}

		//One evolutionary cycle
		private void Evolve()
        {
			//Create mating pool
			_pool.Clear();
			_pool.AddRange(_population);
			_poolCount = _populationSize;
            
            //Evolve offspring
            System.DateTime timestamp = Utility.GetTimestamp();
            for (int i=_elites; i < _populationSize; i++)
            {
                if (_poolCount > 0)
                {
                    Individual parentA = Select(_pool);
                    Individual parentB = Select(_pool);
                    Individual prototype = Select(_pool);

                    //Recombination and Adoption
                    Reproduce(_offspring[i], parentA, parentB, prototype);

                    //Pre-Selection Niching
                    if (_offspring[i].Fitness < parentA.Fitness)
                    {
                        _pool.Remove(parentA);
                        _poolCount -= 1;
                    }
                    if (_offspring[i].Fitness < parentB.Fitness)
                    {
                        _pool.Remove(parentB);
                        _poolCount -= 1;
                    }
                }
                else
                {
                    //Fill the population
                    Reroll(_offspring[i]);
                }
            }
            double duration = Utility.GetElapsedTime(timestamp);

            //Exploit elites sequentially
            for (int i = 0; i < _elites; i++)
            {
                SurviveSequential(i, duration);
            }

			//Reroll elite if exploitation was not successful
			for (int i = 0; i < _elites; i++)
            {
				if (!_improved[i])
                {
					Reroll(_offspring[i]);
				}
			}

			//Swap population and offspring
			Swap(ref _population, ref _offspring);

			//Finalise
			SortByFitness();

			//Check improvement and wipeout criterion
			if (!TryUpdateSolution() && !HasAnyEliteImproved())
            {
				Initialise(_solution);
			}
            else
            {
                ComputeExtinctions();
            }
		}

		//Returns the mutation probability from two parents
		private double GetMutationProbability(Individual parentA, Individual parentB)
        {
			double extinction = 0.5 * (parentA.Extinction + parentB.Extinction);
			double inverse = 1.0 / _dimensionality;
			return extinction * (1.0 - inverse) + inverse;
		}

		//Returns the mutation strength from two parents
		private static double GetMutationStrength(Individual parentA, Individual parentB)
        {
			return 0.5 * (parentA.Extinction + parentB.Extinction);
		}

		//Computes the extinction factors for all individuals
		private void ComputeExtinctions()
        {
			double min = _population[0].Fitness;
			double max = _population[_populationSize-1].Fitness;
			for (int i = 0; i < _populationSize; i++)
            {
				double grading = i/((double) _populationSize - 1);
				_population[i].Extinction = (_population[i].Fitness + min * (grading - 1.0)) / max;
			}
		}

		//Returns whether any elite could be improved by the exploitation
		private bool HasAnyEliteImproved()
        {
			for (int i = 0; i < _elites; i++)
            {
				if (_improved[i])
                {
					return true;
				}
			}
			return false;
		}

		//Tries to improve the evolutionary solution by the population, and returns whether it was successful
		private bool TryUpdateSolution()
        {
			double candidateFitness = _population[0].Fitness;
			if (candidateFitness < _fitness)
            {
				for (int i = 0; i < _dimensionality; i++)
                {
					_solution[i] = _population[0].Genes[i];
				}
				_fitness = candidateFitness;
				return true;
			}

            return false;
        }

		private void SurviveSequential(int index, double timeout)
        {
            //Copy elitist survivor
            Individual survivor = _population[index];
            Individual elite = _offspring[index];
            for (int i = 0; i < _dimensionality; i++)
            {
                elite.Genes[i] = survivor.Genes[i];
                elite.Momentum[i] = survivor.Momentum[i];
            }

            //Exploit
            double fitness = _models[index].ComputeLoss(elite.Genes);
            _optimisers[index].Minimise(elite.Genes, timeout);
            if (_optimisers[index].Value < fitness)
            {
                for (int i = 0; i < _dimensionality; i++)
                {
                    elite.Momentum[i] = _optimisers[index].Solution[i] - elite.Genes[i];
                    elite.Genes[i] = _optimisers[index].Solution[i];
                }
                elite.Fitness = _optimisers[index].Value;        
                _improved[index] = true;
            }
            else
            {
                elite.Fitness = fitness;
                _improved[index] = false;
            }
		}

		//Evolves a new individual
		private void Reproduce(Individual offspring, Individual parentA, Individual parentB, Individual prototype)
        {
            double mutationProbability = GetMutationProbability(parentA, parentB);
			double mutationStrength = GetMutationStrength(parentA, parentB);

			for (int i = 0; i < _dimensionality; i++)
            {
				//Recombination
				_weight = Random.value;
				double momentum = Random.value * parentA.Momentum[i] + Random.value * parentB.Momentum[i];
				offspring.Genes[i] = _weight * parentA.Genes[i] + (1.0 - _weight)*parentB.Genes[i] + momentum;

				//Store
				_gene = offspring.Genes[i];

				//Mutation
                if (Random.value < mutationProbability)
                {
                    offspring.Genes[i] += (Random.value * (_upperBounds[i] - _lowerBounds[i]) + _lowerBounds[i]) * mutationStrength;
                }

				//Adoption
				_weight = Random.value;
				offspring.Genes[i] += 
					_weight * Random.value * (0.5 * (parentA.Genes[i] + parentB.Genes[i]) - offspring.Genes[i])
					+ (1.0 - _weight) * Random.value * (prototype.Genes[i] - offspring.Genes[i]);

				//Project
                if (offspring.Genes[i] < _lowerBounds[i])
                {
                    offspring.Genes[i] = _lowerBounds[i];
                }
                if (offspring.Genes[i] > _upperBounds[i])
                {
                    offspring.Genes[i] = _upperBounds[i];
                }

				//Momentum
				offspring.Momentum[i] = Random.value * momentum + (offspring.Genes[i] - _gene);
			}

			//Fitness
			offspring.Fitness = _model.ComputeLoss(offspring.Genes);
		}

		//Generates a random individual
		private void Reroll(Individual individual)
        {
			for (int i = 0; i < _dimensionality; i++)
            {
                individual.Genes[i] = Random.Range((float)_lowerBounds[i], (float)_upperBounds[i]);
                individual.Momentum[i] = 0.0;
			}
			individual.Fitness = _model.ComputeLoss(individual.Genes);
		}

		//Rank-based selection of an individual
		private Individual Select(List<Individual> pool)
        {
			double rankSum = _poolCount * (_poolCount + 1) / 2.0;
			for (int i = 0; i < _poolCount; i++)
            {
				_probabilities[i] = (_poolCount - i) / rankSum;
			}
			return pool[GetRandomWeightedIndex(_probabilities, _poolCount)];
		}
		
		//Returns a random index with respect to the probability weights
		private static int GetRandomWeightedIndex(double[] probabilities, int count)
        {
			double weightSum = 0.0;
			for (int i = 0; i < count; i++)
            {
				weightSum += probabilities[i];
			}
			double rVal = Random.value * weightSum;
			for (int i = 0; i < count; i++)
            {
				rVal -= probabilities[i];
				if (rVal <= 0.0)
                {
					return i;
				}
			}
			return count-1;
		}

        //Sorts the population by their fitness values (descending)
		private void SortByFitness()
        {
			System.Array.Sort(_population, (a, b) => a.Fitness.CompareTo(b.Fitness));
		}

		//In-place swap by references
		private static void Swap(ref Individual[] a, ref Individual[] b)
        {
			(a, b) = (b, a);
        }

        public Model GetModel()
        {
            return _model;
        }

		//Data class for the individuals
		public class Individual
        {
			public double[] Genes;
			public double[] Momentum;
			public double Fitness;
            public double Extinction;

			public Individual(int dimensionality)
            {
				Genes = new double[dimensionality];
				Momentum = new double[dimensionality];
				Fitness = 0.0;
                Extinction = 0.0;
			}
		}
	}

    ///   Limited-memory Broyden–Fletcher–Goldfarb–Shanno (L-BFGS) optimization method.

    ///   The L-BFGS algorithm is a member of the broad family of quasi-Newton optimization
    ///   methods. L-BFGS stands for 'Limited memory BFGS'. Indeed, L-BFGS uses a limited
    ///   memory variation of the Broyden–Fletcher–Goldfarb–Shanno (BFGS) update to approximate
    ///   the inverse Hessian matrix (denoted by Hk). Unlike the original BFGS method which
    ///   stores a dense  approximation, L-BFGS stores only a few vectors that represent the
    ///   approximation implicitly. Due to its moderate memory requirement, L-BFGS method is
    ///   particularly well suited for optimization problems with a large number of variables.
    ///   L-BFGS never explicitly forms or stores Hk. Instead, it maintains a history of the past
    ///   m updates of the position x and gradient g, where generally the history
    ///   m can be short, often less than 10. These updates are used to implicitly do operations
    ///   requiring the Hk-vector product.

    ///   The framework implementation of this method is based on the original FORTRAN source code
    ///   by Jorge Nocedal (see references below). The original FORTRAN source code of L-BFGS (for
    ///   unconstrained problems) is available at http://www.netlib.org/opt/lbfgs_um.shar and had
    ///   been made available under the public domain.
    /// 
    ///   References:
    ///        http://www.netlib.org/opt/lbfgs_um.shar
    ///        Jorge Nocedal. Limited memory BFGS method for large scale optimization (Fortran source code). 1990.
    ///        Available in http://www.netlib.org/opt/lbfgs_um.shar
    ///        Jorge Nocedal. Updating Quasi-Newton Matrices with Limited Storage. Mathematics of Computation,
    ///        Vol. 35, No. 151, pp. 773--782, 1980.
    ///        Dong C. Liu, Jorge Nocedal. On the limited memory BFGS method for large scale optimization.
    
    public class Bfgs
    {
        public enum Task { None, Start, NewX, Fg, FgLN, FgSt, Abnormal, Convergence, Error, RestartLN, Warning }

        private readonly System.Func<double[], double> _function;
        private readonly System.Func<double[], double[]> _gradient;
        private readonly int _dimensionality;
        public readonly double[] Solution;
        public double Value;
        public double[] LowerBounds;
        public double[] UpperBounds;

        private const int Corrections = 1;
        private const double Factor = 1e+5;
        private const double Tolerance = 0.0;
        private const int Print = 101;

        private double _f;
        private readonly double[] _g;

        private readonly bool[] _lSave;
        private readonly int[] _save;
        private readonly double[] _dSave;

        private readonly int[] _iwa;
        private readonly int[] _nbd;
        private readonly double[] _work;

        private Task _task;
        private Task _cSave;

        private double _newF;
        private double[] _newG;

        public Bfgs(int dimensionality, System.Func<double[], double> function, System.Func<double[], double[]> gradient)
        {
            _dimensionality = dimensionality;
            _function = function;
            _gradient = gradient;
            UpperBounds = new double[_dimensionality];
            LowerBounds = new double[_dimensionality];
            Solution = new double[_dimensionality];

            int totalSize = 2 * Corrections * _dimensionality + 11 * Corrections * Corrections + 5 * _dimensionality + 8 * Corrections;
            _nbd = new int[_dimensionality];
            for (int i = 0; i < _dimensionality; i++)
            {
                _nbd[i] = 2;
            }

            _f = 0.0;
            _g = new double[_dimensionality];
            _lSave = new bool[4];
            _save = new int[44];
            _dSave = new double[29];
            _iwa = new int[3 * _dimensionality];
            _work = new double[totalSize];
            _task = Task.Start;
            _cSave = Task.None;
            _newF = 0;
            _newG = null;
        }

        public void Minimise(double[] values, double timeout)
        {
            for (int i = 0; i < _dimensionality; i++)
            {
                Solution[i] = values[i];
            }

            _f = 0.0;
            System.Array.Clear(_g, 0, _g.Length);
            System.Array.Clear(_lSave, 0, _lSave.Length);
            System.Array.Clear(_save, 0, _save.Length);
            System.Array.Clear(_dSave, 0, _dSave.Length);
            System.Array.Clear(_iwa, 0, _iwa.Length);
            System.Array.Clear(_work, 0, _work.Length);
            _task = Task.Start;
            _cSave = Task.None;
            _newF = 0;
            _newG = null;

            System.DateTime timestamp = Utility.GetTimestamp();
            while(Utility.GetElapsedTime(timestamp) < timeout)
            {
                Setulb(
                    _dimensionality, Corrections, Solution, 0, LowerBounds, 0, UpperBounds, 0, _nbd, 0, ref _f, _g, 0,
                    Factor, Tolerance, _work, 0, _iwa, 0, ref _task, Print, ref _cSave,
                    _lSave, 0, _save, 0, _dSave, 0
                    );

                if (_task is Task.FgLN or Task.FgSt)
                {
                    _newF = _function(Solution);
                    _newG = _gradient(Solution);
                    _f = _newF;
                    for (int i = 0; i < _dimensionality; i++)
                    {
                        _g[i] = _newG[i];
                    }
                }
            }

            Value = _function(Solution);
        }
        
        // 
        // c======================= The end of mainlb =============================
        // 
        // 
        // 
        // c     ************
        // c
        // c     Subroutine active
        // c
        // c     This subroutine initializes iwhere and projects the initial x to
        // c       the feasible set if necessary.
        // c
        // c     iwhere is an integer array of dimension n.
        // c       On entry iwhere is unspecified.
        // c       On exit iwhere(i)=-1  if x(i) has no bounds
        // c                         3   if l(i)=u(i)
        // c                         0   otherwise.
        // c       In cauchy, iwhere is given finer gradations.
        // c
        // c
        // c                           *  *  *
        // c
        // c     NEOS, November 1994. (Latest revision June 1996.)
        // c     Optimization Technology Center.
        // c     Argonne National Laboratory and Northwestern University.
        // c     Written by
        // c                        Ciyou Zhu
        // c     in collaboration with R.H. Byrd, P. Lu-Chen and J. Nocedal.
        // c
        // c
        // c     ************
        // 
        // 
        // c     Initialize nbdd, prjctd, cnstnd and boxed.
        // 
        private static void Active(int n, double[] l, int lOffset, double[] u, int uOffset, int[] nbd,
            int nbdOffset, double[] x, int xOffset, int[] iwhere, int iwhereOffset,
            out bool prjctd, out bool cnstnd, out bool boxed)
        {
            int i;
            prjctd = false;
            cnstnd = false;
            boxed = true;
            // 
            // c     Project the initial x to the feasible set if necessary.
            // 
            {
                for (i = 1; i <= n; i++)
                {
                    if (nbd[i - 1 + nbdOffset] > 0)
                    {
                        if (nbd[i - 1 + nbdOffset] <= 2
                            && x[i - 1 + xOffset] <= l[i - 1 + lOffset])
                        {
                            if (x[i - 1 + xOffset] < l[i - 1 + lOffset])
                            {
                                prjctd = true;
                                x[i - 1 + xOffset] = l[i - 1 + lOffset];
                            }
                        }
                        else if (nbd[i - 1 + nbdOffset] >= 2
                                 && x[i - 1 + xOffset] >= u[i - 1 + uOffset])
                        {
                            if (x[i - 1 + xOffset] > u[i - 1 + uOffset])
                            {
                                prjctd = true;
                                x[i - 1 + xOffset] = u[i - 1 + uOffset];
                            }
                        }
                    }
                }
            }
            // 
            // c     Initialize iwhere and assign values to cnstnd and boxed.
            // 
            {
                for (i = 1; i <= n; i++)
                {
                    if (nbd[i - 1 + nbdOffset] != 2)
                    {
                        boxed = false;
                    }
                    if (nbd[i - 1 + nbdOffset] == 0)
                    {
                        // this variable is always free
                        iwhere[i - 1 + iwhereOffset] = -1;
                        // 
                        // otherwise set x(i)=mid(x(i), u(i), l(i)).
                    }
                    else
                    {
                        cnstnd = true;
                        if (nbd[i - 1 + nbdOffset] == 2 &&
                            u[i - 1 + uOffset] - l[i - 1 + lOffset] <= 0.0)
                        {
                            // this variable is always fixed
                            iwhere[i - 1 + iwhereOffset] = 3;
                        }
                        else
                        {
                            iwhere[i - 1 + iwhereOffset] = 0;
                        }
                    }
                }
            }
        }


        // 
        // c====================== The end of dpofa ===============================
        // 
        // c
        // c
        // c     dtrsl solves systems of the form
        // c
        // c                   t * x = b
        // c     or
        // c                   trans(t) * x = b
        // c
        // c     where t is a triangular matrix of order n. here trans(t)
        // c     denotes the transpose of the matrix t.
        // c
        // c     on entry
        // c
        // c         t         double precision(ldt,n)
        // c                   t contains the matrix of the system. the zero
        // c                   elements of the matrix are not referenced, and
        // c                   the corresponding elements of the array can be
        // c                   used to store other information.
        // c
        // c         ldt       integer
        // c                   ldt is the leading dimension of the array t.
        // c
        // c         n         integer
        // c                   n is the order of the system.
        // c
        // c         b         double precision(n).
        // c                   b contains the right hand side of the system.
        // c
        // c         job       integer
        // c                   job specifies what kind of system is to be solved.
        // c                   if job is
        // c
        // c                        00   solve t*x=b, t lower triangular,
        // c                        01   solve t*x=b, t upper triangular,
        // c                        10   solve trans(t)*x=b, t lower triangular,
        // c                        11   solve trans(t)*x=b, t upper triangular.
        // c
        // c     on return
        // c
        // c         b         b contains the solution, if info .eq. 0.
        // c                   otherwise b is unaltered.
        // c
        // c         info      integer
        // c                   info contains zero if the system is nonsingular.
        // c                   otherwise info contains the index of
        // c                   the first zero diagonal element of t.
        // c
        // c     linpack. this version dated 08/14/78 .
        // c     g. w. stewart, university of maryland, argonne national lab.
        // c
        // c     subroutines and functions
        // c
        // c     blas daxpy,ddot
        // c     fortran mod
        // c
        // c     internal variables
        // c
        // c
        // c     begin block permitting ...exits to 150
        // c
        // c        check for zero diagonal elements.
        // c

        private static void Dtrsl(double[] t, int tOffset, int ldt, int n,
            double[] b, int bOffset, int job, out int info)
        {
            double temp;
            int j;
            int jj;

            for (info = 1; info <= n; info++)
            {
                // ......exit
                if (t[info - 1 + (info - 1) * ldt + tOffset] == 0.0e0)
                {
                    return;
                }
            }

            info = 0;

            // 
            // determine the task and go to it.
            // 
            int @case = 1;

            if (job % 10 != 0)
            {
                @case = 2;
            }

            if (job % 100 / 10 != 0)
            {
                @case += 2;
            }

            switch (@case)
            {
                case 1:
                    goto L20;
                case 2:
                    goto L50;
                case 3:
                    goto L80;
            }

            goto L110;

            //
        // solve t*x=b for t lower triangular
        //
        L20:

            b[1 - 1 + bOffset] /= t[tOffset];

            if (n < 2)
            {
                return;
            }

            for (j = 2; j <= n; j++)
            {
                temp = -b[j - 1 - 1 + bOffset];
                Daxpy(n - j + 1, temp, t, j - 1 + (j - 1
                                                     - 1) * ldt + tOffset, 1, b, j - 1 + bOffset, 1);
                b[j - 1 + bOffset] /= t[j - 1
                                        + (j - 1) * ldt + tOffset];
            }

            return;

        // 
        // solve t*x=b for t upper triangular.
        // 
        L50:

            b[n - 1 + bOffset] /= t[n - 1 + (n - 1) * ldt + tOffset];

            if (n < 2)
            {
                return;
            }

            for (jj = 2; jj <= n; jj++)
            {
                j = n - jj + 1;
                temp = -b[j + 1 - 1 + bOffset];
                Daxpy(j, temp,
                    t, 1 - 1 + (j + 1 - 1) * ldt + tOffset, 1,
                    b, 1 - 1 + bOffset, 1);

                b[j - 1 + bOffset] /= t[j - 1 + (j - 1) * ldt + tOffset];
            }

            return;
        // 
        // solve trans(t)*x=b for t lower triangular.
        // 
        L80:
            b[n - 1 + bOffset] /= t[n - 1 + (n - 1) * ldt + tOffset];

            if (n < 2)
            {
                return;
            }

            for (jj = 2; jj <= n; jj++)
            {
                j = n - jj + 1;
                b[j - 1 + bOffset] -= Ddot(jj - 1, t, j + 1 - 1 + (j - 1) * ldt + tOffset,
                    1, b, j + 1 - 1 + bOffset, 1);

                b[j - 1 + bOffset] /= t[j - 1
                                        + (j - 1) * ldt + tOffset];
            }

            return;

        // 
        // solve trans(t)*x=b for t upper triangular.
        // 
        L110:
            b[1 - 1 + bOffset] /= t[tOffset];

            if (n < 2)
            {
                return;
            }

            for (j = 2; j <= n; j++)
            {
                b[j - 1 + bOffset] -= Ddot(j - 1, t, 1 - 1 + (j - 1) * ldt + tOffset,
                    1, b, 1 - 1 + bOffset, 1);

                b[j - 1 + bOffset] /= t[j - 1 + (j - 1) * ldt + tOffset];
            }
        }

        // 
        // c======================= The end of active =============================
        // 
        // 
        // 
        // c     ************
        // c
        // c     Subroutine bmv
        // c
        // c     This subroutine computes the product of the 2m x 2m middle matrix 
        // c       in the compact L-BFGS formula of B and a 2m vector v;  
        // c       it returns the product in p.
        // c       
        // c     m is an integer variable.
        // c       On entry m is the maximum number of variable metric corrections

        // c         used to define the limited memory matrix.
        // c       On exit m is unchanged.
        // c
        // c     sy is a double precision array of dimension m x m.
        // c       On entry sy specifies the matrix S'Y.
        // c       On exit sy is unchanged.
        // c
        // c     wt is a double precision array of dimension m x m.
        // c       On entry wt specifies the upper triangular matrix J' which is 
        // c         the Cholesky factor of (thetaS'S+LD^(-1)L').
        // c       On exit wt is unchanged.
        // c
        // c     col is an integer variable.
        // c       On entry col specifies the number of s-vectors (or y-vectors)
        // c         stored in the compact L-BFGS formula.
        // c       On exit col is unchanged.
        // c
        // c     v is a double precision array of dimension 2col.
        // c       On entry v specifies vector v.
        // c       On exit v is unchanged.
        // c
        // c     p is a double precision array of dimension 2col.
        // c       On entry p is unspecified.
        // c       On exit p is the product Mv.
        // c
        // c     info is an integer variable.
        // c       On entry info is unspecified.
        // c       On exit info = 0       for normal return,
        // c                    = nonzero for abnormal return when the system
        // c                                to be solved by dtrsl is singular.
        // c
        // c     Subprograms called:
        // c
        // c       Linpack ... 
        // c
        // c
        // c                           *  *  *
        // c
        // c     NEOS, November 1994. (Latest revision June 1996.)
        // c     Optimization Technology Center.
        // c     Argonne National Laboratory and Northwestern University.
        // c     Written by
        // c                        Ciyou Zhu
        // c     in collaboration with R.H. Byrd, P. Lu-Chen and J. Nocedal.
        // c
        // c
        // c     ************
        // 
        // 
        private static void Bmv(int m, double[] sy, int syOffset, double[] wt, int wtOffset,
            int col, double[] v, int vOffset, double[] p, int pOffset, ref int info)
        {

            int i;
            int k;
            int i2;
            double sum;

            if (col == 0)
                return;

            // 
            // PART I: solve [  D^(1/2)      O ] [ p1 ] = [ v1 ]
            //               [ -L*D^(-1/2)   J ] [ p2 ]   [ v2 ].
            // 
            //  solve Jp2=v2+LD^(-1)v1.
            //
            p[col + 1 - 1 + pOffset] = v[col + 1 - 1 + vOffset];
            for (i = 2; i <= col; i++)
            {
                i2 = col + i;
                sum = 0.0e0;
                for (k = 1; k <= i - 1; k++)
                {
                    sum += sy[i - 1 + (k - 1) * m
                                    + syOffset] * v[k - 1 + vOffset] / sy[k - 1
                                                                          + (k - 1) * m + syOffset];
                }

                p[i2 - 1 + pOffset] = v[i2 - 1 + vOffset] + sum;
            }

            // Solve the triangular system
            Dtrsl(wt, wtOffset, m, col, p,
                col + 1 - 1 + pOffset, 11, out info);

            // 
            // solve D^(1/2)p1=v1.
            for (i = 1; i <= col; i++)
            {
                p[i - 1 + pOffset] = v[i - 1 + vOffset]
                                     / System.Math.Sqrt(sy[i - 1 + (i - 1) * m + syOffset]);
            }

            // 
            //  PART II: solve [ -D^(1/2)   D^(-1/2)*L'  ] [ p1 ] = [ p1 ]
            //                 [  0         J'           ] [ p2 ]   [ p2 ]. 
            // 
            //    solve J^Tp2=p2. 
            //
            Dtrsl(wt, wtOffset, m, col,
                p, col + 1 - 1 + pOffset, 01, out info);

            // 
            // compute p1 = -D^(-1/2)(p1-D^(-1/2)L'p2)
            //            = -D^(-1/2)p1+D^(-1)L'p2.  
            for (i = 1; i <= col; i++)
            {
                p[i - 1 + pOffset] = -(p[i - 1 + pOffset]
                                       / System.Math.Sqrt(sy[i - 1 + (i - 1) * m + syOffset]));
            }

            for (i = 1; i <= col; i++)
            {
                sum = 0.0e0;
                for (k = i + 1; k <= col; k++)
                {
                    sum += sy[k - 1 + (i - 1) * m + syOffset]
                        * p[col + k - 1 + pOffset] / sy[i - 1 + (i - 1) * m + syOffset];
                }

                p[i - 1 + pOffset] += sum;
            }
        }

        // 
        // c======================== The end of bmv ===============================
        // 
        // 
        // c     ************
        // c
        // c     Subroutine cauchy
        // c
        // c     For given x, l, u, g (with sbgnrm > 0), and a limited memory
        // c       BFGS matrix B defined in terms of matrices WY, WS, WT, and
        // c       scalars head, col, and theta, this subroutine computes the
        // c       generalized Cauchy point (GCP), defined as the first local
        // c       minimizer of the quadratic
        // c
        // c                  Q(x + s) = g's + 1/2 s'Bs
        // c
        // c       along the projected gradient direction P(x-tg,l,u).
        // c       The routine returns the GCP in xcp. 
        // c       
        // c     n is an integer variable.
        // c       On entry n is the dimension of the problem.
        // c       On exit n is unchanged.
        // c
        // c     x is a double precision array of dimension n.
        // c       On entry x is the starting point for the GCP computation.
        // c       On exit x is unchanged.
        // c
        // c     l is a double precision array of dimension n.
        // c       On entry l is the lower bound of x.
        // c       On exit l is unchanged.
        // c
        // c     u is a double precision array of dimension n.
        // c       On entry u is the upper bound of x.
        // c       On exit u is unchanged.
        // c
        // c     nbd is an integer array of dimension n.
        // c       On entry nbd represents the type of bounds imposed on the
        // c         variables, and must be specified as follows:
        // c         nbd(i)=0 if x(i) is unbounded,
        // c                1 if x(i) has only a lower bound,
        // c                2 if x(i) has both lower and upper bounds, and
        // c                3 if x(i) has only an upper bound. 
        // c       On exit nbd is unchanged.
        // c
        // c     g is a double precision array of dimension n.
        // c       On entry g is the gradient of f(x).  g must be a nonzero vector.
        // c       On exit g is unchanged.
        // c
        // c     iorder is an integer working array of dimension n.
        // c       iorder will be used to store the breakpoints in the piecewise
        // c       linear path and free variables encountered. On exit,
        // c         iorder(1),...,iorder(nleft) are indices of breakpoints
        // c                                which have not been encountered; 
        // c         iorder(nleft+1),...,iorder(nbreak) are indices of
        // c                                     encountered breakpoints; and
        // c         iorder(nfree),...,iorder(n) are indices of variables which
        // c                 have no bound constraits along the search direction.
        // c
        // c     iwhere is an integer array of dimension n.
        // c       On entry iwhere indicates only the permanently fixed (iwhere=3)

        // c       or free (iwhere= -1) components of x.
        // c       On exit iwhere records the status of the current x variables.
        // c       iwhere(i)=-3  if x(i) is free and has bounds, but is not moved
        // c                 0   if x(i) is free and has bounds, and is moved
        // c                 1   if x(i) is fixed at l(i), and l(i) .ne. u(i)
        // c                 2   if x(i) is fixed at u(i), and u(i) .ne. l(i)
        // c                 3   if x(i) is always fixed, i.e.,  u(i)=x(i)=l(i)
        // c                 -1  if x(i) is always free, i.e., it has no bounds.
        // c
        // c     t is a double precision working array of dimension n. 
        // c       t will be used to store the break points.
        // c
        // c     d is a double precision array of dimension n used to store
        // c       the Cauchy direction P(x-tg)-x.
        // c
        // c     xcp is a double precision array of dimension n used to return the

        // c       GCP on exit.
        // c
        // c     m is an integer variable.
        // c       On entry m is the maximum number of variable metric corrections 
        // c         used to define the limited memory matrix.
        // c       On exit m is unchanged.
        // c
        // c     ws, wy, sy, and wt are double precision arrays.
        // c       On entry they store information that defines the
        // c                             limited memory BFGS matrix:
        // c         ws(n,m) stores S, a set of s-vectors;
        // c         wy(n,m) stores Y, a set of y-vectors;
        // c         sy(m,m) stores S'Y;
        // c         wt(m,m) stores the
        // c                 Cholesky factorization of (theta*S'S+LD^(-1)L').
        // c       On exit these arrays are unchanged.
        // c
        // c     theta is a double precision variable.
        // c       On entry theta is the scaling factor specifying B_0 = theta I.
        // c       On exit theta is unchanged.
        // c
        // c     col is an integer variable.
        // c       On entry col is the actual number of variable metric
        // c         corrections stored so far.
        // c       On exit col is unchanged.
        // c
        // c     head is an integer variable.
        // c       On entry head is the location of the first s-vector (or y-vector
        // c         in S (or Y).
        // c       On exit col is unchanged.
        // c
        // c     p is a double precision working array of dimension 2m.
        // c       p will be used to store the vector p = W^(T)d.
        // c
        // c     c is a double precision working array of dimension 2m.
        // c       c will be used to store the vector c = W^(T)(xcp-x).
        // c
        // c     wbp is a double precision working array of dimension 2m.
        // c       wbp will be used to store the row of W corresponding
        // c         to a breakpoint.
        // c
        // c     v is a double precision working array of dimension 2m.
        // c
        // c     nseg is an integer variable.
        // c       On exit nseg records the number of quadratic segments explored
        // c         in searching for the GCP.
        // c
        // c     sg and yg are double precision arrays of dimension m.
        // c       On entry sg  and yg store S'g and Y'g correspondingly.
        // c       On exit they are unchanged. 
        // c 
        // c     iprint is an INTEGER variable that must be set by the user.
        // c       It controls the frequency and type of output generated:
        // c        iprint<0    no output is generated;
        // c        iprint=0    print only one line at the last iteration;
        // c        0<iprint<99 print also f and |proj g| every iprint iterations;

        // c        iprint=99   print details of every iteration except n-vectors;

        // c        iprint=100  print also the changes of active set and final x;
        // c        iprint>100  print details of every iteration including x and g;
        // c       When iprint > 0, the file iterate.dat will be created to
        // c                        summarize the iteration.
        // c
        // c     sbgnrm is a double precision variable.
        // c       On entry sbgnrm is the norm of the projected gradient at x.
        // c       On exit sbgnrm is unchanged.
        // c
        // c     info is an integer variable.
        // c       On entry info is 0.
        // c       On exit info = 0       for normal return,
        // c                    = nonzero for abnormal return when the the system
        // c                              used in routine bmv is singular.
        // c
        // c     Subprograms called:
        // c 
        // c       L-BFGS-B Library ... hpsolb, bmv.
        // c
        // c       Linpack ... dscal dcopy, 
        // c
        // c
        // c     References:
        // c
        // c       [1] R. H. Byrd, P. Lu, J. Nocedal and C. Zhu, ``A limited
        // c       memory algorithm for bound constrained optimization'',
        // c       SIAM J. Scientific Computing 16 (1995), no. 5, pp. 1190--1208.
        // c
        // c       [2] C. Zhu, R.H. Byrd, P. Lu, J. Nocedal, ``L-BFGS-B: FORTRAN
        // c       Subroutines for Large Scale Bound Constrained Optimization''
        // c       Tech. Report, NAM-11, EECS Department, Northwestern University,

        // c       1994.
        // c
        // c       (Postscript files of these papers are available via anonymous
        // c        ftp to eecs.nwu.edu in the directory pub/lbfgs/lbfgs_bcm.)
        // c
        // c                           *  *  *
        // c
        // c     NEOS, November 1994. (Latest revision June 1996.)
        // c     Optimization Technology Center.
        // c     Argonne National Laboratory and Northwestern University.
        // c     Written by
        // c                        Ciyou Zhu
        // c     in collaboration with R.H. Byrd, P. Lu-Chen and J. Nocedal.
        // c
        // c
        // c     ************
        // 
        // 
        // c     Check the status of the variables, reset iwhere(i) if necessary;
        // c       compute the Cauchy direction d and the breakpoints t; initialize
        // c       the derivative f1 and the vector p = W'd (for theta = 1).
        // 
        private static void Cauchy(int n, double[] x, int xOffset, double[] l, int lOffset,
            double[] u, int uOffset, int[] nbd, int nbdOffset, double[] g, int gOffset,
            int[] iorder, int iorderOffset, int[] iwhere, int iwhereOffset, double[] t, int tOffset,
            double[] d, int dOffset, double[] xcp, int xcpOffset, int m,
            double[] wy, int wyOffset, double[] ws, int wsOffset, double[] sy, int syOffset,
            double[] wt, int wtOffset, double theta, int col,
            int head, double[] p, int pOffset, double[] c, int cOffset,
            double[] wbp, int wbpOffset, double[] v, int vOffset, ref int nseg, double sbgnrm, ref int info, double epsmch)
        {
            int i;
            int j;
            int pointr;
            int ibp;
            double zibp;
            double tu = 0.0d;
            double tl = 0.0d;

            if (sbgnrm <= 0.0)
            {
                Dcopy(n, x, xOffset, 1, xcp, xcpOffset, 1);
                return;
            }

            bool bnded = true;
            int nfree = n + 1;
            int nbreak = 0;
            int ibkmin = 0;
            double bkmin = 0.0;
            int col2 = 2 * col;
            double f1 = 0.0;
            // 
            // c     We set p to zero and build it up as we determine d.
            // 
            for (i = 1; i <= col2; i++)
            {
                p[i - 1 + pOffset] = 0.0;
            }

            // 
            // c     In the following loop we determine for each variable its bound
            // c        status and its breakpoint, and update p accordingly.
            // c        Smallest breakpoint is identified.
            // 
            for (i = 1; i <= n; i++)
            {
                double neggi = -g[i - 1 + gOffset];

                if (iwhere[i - 1 + iwhereOffset] != 3
                    && iwhere[i - 1 + iwhereOffset] != -1)
                {
                    // c             if x(i) is not a constant and has bounds,
                    // c             compute the difference between x(i) and its bounds.
                    if (nbd[i - 1 + nbdOffset] <= 2)
                    {
                        tl = x[i - 1 + xOffset] - l[i - 1 + lOffset];
                    }

                    if (nbd[i - 1 + nbdOffset] >= 2)
                    {
                        tu = u[i - 1 + uOffset] - x[i - 1 + xOffset];
                    }

                    // 
                    // c           If a variable is close enough to a bound
                    // c             we treat it as at bound.
                    bool xlower = nbd[i - 1 + nbdOffset] <= 2 && tl <= 0.0;
                    bool xupper = nbd[i - 1 + nbdOffset] >= 2 && tu <= 0.0;
                    // 
                    // c              reset iwhere(i).
                    iwhere[i - 1 + iwhereOffset] = 0;
                    if (xlower)
                    {
                        if (neggi <= 0.0)
                        {
                            iwhere[i - 1 + iwhereOffset] = 1;
                        }
                    }
                    else if (xupper)
                    {
                        if (neggi >= 0.0)
                        {
                            iwhere[i - 1 + iwhereOffset] = 2;
                        }
                    }
                    else
                    {
                        if (System.Math.Abs(neggi) <= 0.0)
                        {
                            iwhere[i - 1 + iwhereOffset] = -3;
                        }
                    }
                }

                pointr = head;
                if (iwhere[i - 1 + iwhereOffset] != 0 && iwhere[i - 1 + iwhereOffset] != -1)
                {
                    d[i - 1 + dOffset] = 0.0;
                }
                else
                {
                    d[i - 1 + dOffset] = neggi;
                    f1 -= neggi * neggi;

                    // calculate p := p - W'e_i* (g_i).
                    for (j = 1; j <= col; j++)
                    {
                        p[j - 1 + pOffset] += wy[i - 1 + (pointr - 1) * n + wyOffset] * neggi;
                        p[col + j - 1 + pOffset] += ws[i - 1 + (pointr - 1) * n + wsOffset] * neggi;
                        pointr = pointr % m + 1;
                    }

                    if (nbd[i - 1 + nbdOffset] <= 2
                        && nbd[i - 1 + nbdOffset] != 0 && neggi < 0.0)
                    {
                        // x(i) + d(i) is bounded; compute t(i).

                        nbreak += 1;
                        iorder[nbreak - 1 + iorderOffset] = i;
                        t[nbreak - 1 + tOffset] = tl / -neggi;
                        if (nbreak == 1 || t[nbreak - 1 + tOffset] < bkmin)
                        {
                            bkmin = t[nbreak - 1 + tOffset];
                            ibkmin = nbreak;
                        }
                    }
                    else if (nbd[i - 1 + nbdOffset] >= 2 && neggi > 0.0)
                    {
                        // x(i) + d(i) is bounded; compute t(i).

                        nbreak += 1;
                        iorder[nbreak - 1 + iorderOffset] = i;
                        t[nbreak - 1 + tOffset] = tu / neggi;
                        if (nbreak == 1 || t[nbreak - 1 + tOffset] < bkmin)
                        {
                            bkmin = t[nbreak - 1 + tOffset];
                            ibkmin = nbreak;
                        }
                    }
                    else
                    {
                        // x(i) + d(i) is not bounded.
                        nfree -= 1;
                        iorder[nfree - 1 + iorderOffset] = i;
                        if (System.Math.Abs(neggi) > 0.0)
                        {
                            bnded = false;
                        }
                    }
                }
            }

            // 
            // The indices of the nonzero components of d are now stored
            // in iorder(1),...,iorder(nbreak) and iorder(nfree),...,iorder(n).
            // The smallest of the nbreak breakpoints is in t(ibkmin)=bkmin.
            // 
            if (theta != 1.0)
            {
                // complete the initialization of p for theta not= one.
                Dscal(col, theta, p, col + 1 - 1 + pOffset, 1);
            }

            // 
            // c     Initialize GCP xcp = x.
            // 
            Dcopy(n, x, xOffset, 1, xcp, xcpOffset, 1);

            if (nbreak == 0 && nfree == n + 1)
            {
                // is a zero vector, return with the initial xcp as GCP.
                return;
            }

            // 
            // c     Initialize c = W'(xcp - x) = 0.
            // 
            for (j = 1; j <= col2; j++)
            {
                c[j - 1 + cOffset] = 0.0;
            }

            // 
            // c     Initialize derivative f2.
            // 
            double f2 = -(theta * f1);
            double f2Org = f2;
            if (col > 0)
            {
                Bmv(m, sy, syOffset, wt, wtOffset,
                    col, p, pOffset, v, vOffset, ref info);

                f2 -= Ddot(col2, v, vOffset, 1, p, pOffset, 1);
            }

            double dtm = -(f1 / f2);
            double tsum = 0.0;
            nseg = 1;

            // 
            // c     If there are no breakpoints, locate the GCP and return. 
            // 
            if (nbreak == 0)
                goto L888;

            int nleft = nbreak;
            int iter = 1;
            double tj = 0.0;

            // 
            // c------------------- the beginning of the loop -------------------------
            // 
            L777:
            // 
            //     Find the next smallest breakpoint;
            //      compute dt = t(nleft) - t(nleft + 1).
            // 
            double tj0 = tj;

            if (iter == 1)
            {
                // Since we already have the smallest breakpoint we need not do
                // heapsort yet. Often only one breakpoint is used and the
                // cost of heapsort is avoided.
                tj = bkmin;
                ibp = iorder[ibkmin - 1 + iorderOffset];
            }
            else
            {
                if (iter == 2)
                {
                    // Replace the already used smallest breakpoint with the
                    // breakpoint numbered nbreak > nlast, before heapsort call.

                    if (ibkmin != nbreak)
                    {
                        t[ibkmin - 1 + tOffset] = t[nbreak - 1 + tOffset];
                        iorder[ibkmin - 1 + iorderOffset] = iorder[nbreak - 1 + iorderOffset];
                    }
                    // Update heap structure of breakpoints
                    //   (if iter=2, initialize heap).
                }
                Hpsolb(nleft, t, tOffset, iorder, iorderOffset, iter - 2);
                tj = t[nleft - 1 + tOffset];
                ibp = iorder[nleft - 1 + iorderOffset];
            }

            // 
            double dt = tj - tj0;
            // 

            // 
            // If a minimizer is within this interval, locate the GCP and return.
            // 
            if (dtm < dt)
                goto L888;

            // 
            // Otherwise fix one variable and
            //   reset the corresponding component of d to zero.
            // 
            tsum += dt;
            nleft -= 1;
            iter += 1;
            double dibp = d[ibp - 1 + dOffset];
            d[ibp - 1 + dOffset] = 0.0;

            if (dibp > 0.0)
            {
                zibp = u[ibp - 1 + uOffset] - x[ibp - 1 + xOffset];
                xcp[ibp - 1 + xcpOffset] = u[ibp - 1 + uOffset];
                iwhere[ibp - 1 + iwhereOffset] = 2;
            }
            else
            {
                zibp = l[ibp - 1 + lOffset] - x[ibp - 1 + xOffset];
                xcp[ibp - 1 + xcpOffset] = l[ibp - 1 + lOffset];
                iwhere[ibp - 1 + iwhereOffset] = 1;
            }

            if (nleft == 0 && nbreak == n)
            {
                // all n variables are fixed,
                // return with xcp as GCP.
                dtm = dt;
                goto L999;
            }

            // 
            // Update the derivative information.
            // 
            nseg += 1;
            double dibp2 = System.Math.Pow(dibp, 2);

            // 
            // Update f1 and f2.
            // 
            // temporarily set f1 and f2 for col=0.
            //
            f1 = f1 + dt * f2 + dibp2 - theta * dibp * zibp;
            f2 -= theta * dibp2;
            // 
            if (col > 0)
            {
                // update c = c + dt*p.
                Daxpy(col2, dt, p, pOffset, 1, c, cOffset, 1);

                // choose wbp,
                // the row of W corresponding to the breakpoint encountered.
                pointr = head;
                {
                    for (j = 1; j <= col; j++)
                    {
                        wbp[j - 1 + wbpOffset] = wy[ibp - 1
                                                      + (pointr - 1) * n + wyOffset];

                        wbp[col + j - 1 + wbpOffset] = theta * ws[ibp
                            - 1 + (pointr - 1) * n + wsOffset];

                        pointr = pointr % m + 1;
                    }
                }

                // compute (wbp)Mc, (wbp)Mp, and (wbp)M(wbp)'.
                Bmv(m, sy, syOffset, wt, wtOffset, col, wbp,
                    wbpOffset, v, vOffset, ref info);

                double wmc = Ddot(col2, c, cOffset, 1, v, vOffset, 1);
                double wmp = Ddot(col2, p, pOffset, 1, v, vOffset, 1);
                double wmw = Ddot(col2, wbp, wbpOffset, 1, v, vOffset, 1);

                // update p = p - dibp*wbp. 
                Daxpy(col2, -dibp, wbp, wbpOffset, 1, p, pOffset, 1);

                // complete updating f1 and f2 while col > 0.
                f1 += dibp * wmc;
                f2 = f2 + 2.0e0 * dibp * wmp - dibp2 * wmw;
            }


            f2 = System.Math.Max(epsmch * f2Org, f2);

            if (nleft > 0)
            {
                dtm = -(f1 / f2);
                goto L777;
                // to repeat the loop for unsearched intervals. 
            }

            if (bnded)
            {
                dtm = 0.0;
            }
            else
            {
                dtm = -(f1 / f2);
            }

            // 
        // c------------------- the end of the loop -------------------------------
        // 
        L888:
        
            if (dtm <= 0.0)
            {
                dtm = 0.0;
            }
            tsum += dtm;

            // 
            // Move free variables (i.e., the ones w/o breakpoints) and 
            //   the variables whose breakpoints haven't been reached.
            // 
            Daxpy(n, tsum, d, dOffset, 1, xcp, xcpOffset, 1);
        // 
        L999:
            // 
            // Update c = c + dtm*p = W'(x^c - x) 
            //   which will be used in computing r = Z'(B(x^c - x) + g).
            // 
            if (col > 0)
            {
                Daxpy(col2, dtm, p, pOffset, 1, c, cOffset, 1);
            }
        }

        // 
        // c====================== The end of cauchy ==============================
        // 
        // 
        // 
        // c     ************
        // c
        // c     Subroutine cmprlb 
        // c
        // c       This subroutine computes r=-Z'B(xcp-xk)-Z'g by using 
        // c         wa(2m+1)=W'(xcp-x) from subroutine cauchy.
        // c
        // c     Subprograms called:
        // c
        // c       L-BFGS-B Library ... bmv.
        // c
        // c
        // c                           *  *  *
        // c
        // c     NEOS, November 1994. (Latest revision June 1996.)
        // c     Optimization Technology Center.
        // c     Argonne National Laboratory and Northwestern University.
        // c     Written by
        // c                        Ciyou Zhu
        // c     in collaboration with R.H. Byrd, P. Lu-Chen and J. Nocedal.
        // c
        // c
        // c     ************
        // 
        // 
        private static void Cmprlb(int n, int m, double[] x, int xOffset, double[] g, int gOffset, double[] ws, int wsOffset, double[] wy, int wyOffset, double[] sy, int syOffset,
        double[] wt, int wtOffset, double[] z, int zOffset, double[] r, int rOffset,
        double[] wa, int waOffset, int[] index, int indexOffset, double theta,
        int col, int head, int nfree, bool cnstnd, ref int info)
        {

            int i = 0;
            int j = 0;
            int k = 0;
            int pointr = 0;
            double a1 = 0.0d;
            double a2 = 0.0d;

            if (!cnstnd && col > 0)
            {
                {
                    for (i = 1; i <= n; i++)
                    {
                        r[i - 1 + rOffset] = -g[i - 1 + gOffset];
                    }
                }
            }
            else
            {
                {
                    for (i = 1; i <= nfree; i++)
                    {
                        k = index[i - 1 + indexOffset];
                        r[i - 1 + rOffset] = -(theta * (z[k - 1
                                                            + zOffset] - x[k - 1 + xOffset])) - g[k - 1 + gOffset];
                    }
                }

                Bmv(m, sy, syOffset, wt, wtOffset, col, wa,
                    2 * m + 1 - 1 + waOffset, wa, 1 - 1 + waOffset, ref info);

                pointr = head;

                {
                    for (j = 1; j <= col; j++)
                    {
                        a1 = wa[j - 1 + waOffset];
                        a2 = theta * wa[col + j - 1 + waOffset];
                        {
                            for (i = 1; i <= nfree; i++)
                            {
                                k = index[i - 1 + indexOffset];
                                r[i - 1 + rOffset] = r[i - 1 + rOffset]
                                                       + wy[k - 1 + (pointr - 1) * n + wyOffset] * a1
                                                       + ws[k - 1 + (pointr - 1) * n + wsOffset] * a2;
                            }
                        }

                        pointr = pointr % m + 1;
                    }
                }
            }
        }

        // c====================== The end of subsm ===============================
        // 
        // c     **********
        // c
        // c     Subroutine dcsrch
        // c
        // c     This subroutine finds a step that satisfies a sufficient
        // c     decrease condition and a curvature condition.
        // c
        // c     Each call of the subroutine updates an interval with 
        // c     endpoints stx and sty. The interval is initially chosen 
        // c     so that it contains a minimizer of the modified function
        // c
        // c           psi(stp) = f(stp) - f(0) - ftol*stp*f'(0).
        // c
        // c     If psi(stp) <= 0 and f'(stp) >= 0 for some step, then the
        // c     interval is chosen so that it contains a minimizer of f. 
        // c
        // c     The algorithm is designed to find a step that satisfies 
        // c     the sufficient decrease condition 
        // c
        // c           f(stp) <= f(0) + ftol*stp*f'(0),
        // c
        // c     and the curvature condition
        // c
        // c           abs(f'(stp)) <= gtol*abs(f'(0)).
        // c
        // c     If ftol is less than gtol and if, for example, the function
        // c     is bounded below, then there is always a step which satisfies
        // c     both conditions. 
        // c
        // c     If no step can be found that satisfies both conditions, then 
        // c     the algorithm stops with a warning. In this case stp only 
        // c     satisfies the sufficient decrease condition.
        // c
        // c     A typical invocation of dcsrch has the following outline:
        // c
        // c     task = 'START'
        // c  10 continue
        // c        call dcsrch( ... )
        // c        if (task .eq. 'FG') then
        // c           Evaluate the function and the gradient at stp 
        // c           goto 10
        // c           end if
        // c
        // c     NOTE: The user must no alter work arrays between calls.
        // c
        // c     The subroutine statement is
        // c
        // c        subroutine dcsrch(f,g,stp,ftol,gtol,xtol,stpmin,stpmax,
        // c                          task,isave,dsave)
        // c     where
        // c
        // c       f is a double precision variable.
        // c         On initial entry f is the value of the function at 0.
        // c            On subsequent entries f is the value of the 
        // c            function at stp.
        // c         On exit f is the value of the function at stp.
        // c
        // c       g is a double precision variable.
        // c         On initial entry g is the derivative of the function at 0.
        // c            On subsequent entries g is the derivative of the 
        // c            function at stp.
        // c         On exit g is the derivative of the function at stp.
        // c
        // c       stp is a double precision variable. 
        // c         On entry stp is the current estimate of a satisfactory 
        // c            step. On initial entry, a positive initial estimate 
        // c            must be provided. 
        // c         On exit stp is the current estimate of a satisfactory step
        // c            if task = 'FG'. If task = 'CONV' then stp satisfies
        // c            the sufficient decrease and curvature condition.
        // c
        // c       ftol is a double precision variable.
        // c         On entry ftol specifies a nonnegative tolerance for the 
        // c            sufficient decrease condition.
        // c         On exit ftol is unchanged.
        // c
        // c       gtol is a double precision variable.
        // c         On entry gtol specifies a nonnegative tolerance for the 
        // c            curvature condition. 
        // c         On exit gtol is unchanged.
        // c
        // c       xtol is a double precision variable.
        // c         On entry xtol specifies a nonnegative relative tolerance
        // c            for an acceptable step. The subroutine exits with a
        // c            warning if the relative difference between sty and stx
        // c            is less than xtol.
        // c         On exit xtol is unchanged.
        // c
        // c       stpmin is a double precision variable.
        // c         On entry stpmin is a nonnegative lower bound for the step.
        // c         On exit stpmin is unchanged.
        // c
        // c       stpmax is a double precision variable.
        // c         On entry stpmax is a nonnegative upper bound for the step.
        // c         On exit stpmax is unchanged.
        // c
        // c       task is a character variable of length at least 60.
        // c         On initial entry task must be set to 'START'.
        // c         On exit task indicates the required action:
        // c
        // c            If task(1:2) = 'FG' then evaluate the function and 
        // c            derivative at stp and call dcsrch again.
        // c
        // c            If task(1:4) = 'CONV' then the search is successful.
        // c
        // c            If task(1:4) = 'WARN' then the subroutine is not able
        // c            to satisfy the convergence conditions. The exit value of
        // c            stp contains the best point found during the search.
        // c
        // c            If task(1:5) = 'ERROR' then there is an error in the
        // c            input arguments.
        // c
        // c         On exit with convergence, a warning or an error, the
        // c            variable task contains additional information.
        // c
        // c       isave is an integer work array of dimension 2.
        // c         
        // c       dsave is a double precision work array of dimension 13.
        // c
        // c     Subprograms called
        // c
        // c       MINPACK-2 ... dcstep
        // c
        // c     MINPACK-1 Project. June 1983.
        // c     Argonne National Laboratory. 
        // c     Jorge J. More' and David J. Thuente.
        // c
        // c     MINPACK-2 Project. October 1993.
        // c     Argonne National Laboratory and University of Minnesota. 
        // c     Brett M. Averick, Richard G. Carter, and Jorge J. More'. 
        // c
        // c     **********
        // 
        // 
        // c     Initialization block.
        // 
        private static void Dcsrch(double f, double g, ref double stp, double ftol,
        double gtol, double xtol, double stpmin, double stpmax, ref Task task,
        int[] isave, int isaveOffset, double[] dsave, int dsaveOffset)
        {
            bool brackt = false;
            int stage = 0;
            double finit = 0.0d;
            double ftest = 0.0d;
            double fm = 0.0d;
            double fx = 0.0d;
            double fxm = 0.0d;
            double fy = 0.0d;
            double fym = 0.0d;
            double ginit = 0.0d;
            double gtest = 0.0d;
            double gm = 0.0d;
            double gx = 0.0d;
            double gxm = 0.0d;
            double gy = 0.0d;
            double gym = 0.0d;
            double stx = 0.0d;
            double sty = 0.0d;
            double stmin = 0.0d;
            double stmax = 0.0d;
            double width = 0.0d;
            double width1 = 0.0d;

            if (task == Task.Start)
            {
                // 
                // Initialize local variables.
                // 
                brackt = false;
                stage = 1;
                finit = f;
                ginit = g;
                gtest = ftol * ginit;
                width = stpmax - stpmin;
                width1 = width / 0.5;

                // 
                // The variables stx, fx, gx contain the values of the step, 
                // function, and derivative at the best step. 
                // The variables sty, fy, gy contain the value of the step, 
                // function, and derivative at sty.
                // The variables stp, f, g contain the values of the step, 
                // function, and derivative at stp.
                // 
                stx = 0.0;
                fx = finit;
                gx = ginit;
                sty = 0.0;
                fy = finit;
                gy = ginit;
                stmin = 0.0;
                stmax = stp + 4.0 * stp;
                task = Task.Fg;
                goto L1000;
            }

            // Restore local variables.
            if (isave[1 - 1 + isaveOffset] == 1)
            {
                brackt = true;
            }
            else
            {
                brackt = false;
            }
            stage = isave[2 - 1 + isaveOffset];
            ginit = dsave[1 - 1 + dsaveOffset];
            gtest = dsave[2 - 1 + dsaveOffset];
            gx = dsave[3 - 1 + dsaveOffset];
            gy = dsave[4 - 1 + dsaveOffset];
            finit = dsave[5 - 1 + dsaveOffset];
            fx = dsave[6 - 1 + dsaveOffset];
            fy = dsave[7 - 1 + dsaveOffset];
            stx = dsave[8 - 1 + dsaveOffset];
            sty = dsave[9 - 1 + dsaveOffset];
            stmin = dsave[10 - 1 + dsaveOffset];
            stmax = dsave[11 - 1 + dsaveOffset];
            width = dsave[12 - 1 + dsaveOffset];
            width1 = dsave[13 - 1 + dsaveOffset];

            // 
            // c     If psi(stp) <= 0 and f'(stp) >= 0 for some step, then the
            // c     algorithm enters the second stage.
            // 
            ftest = finit + stp * gtest;
            if (stage == 1 && f <= ftest && g >= 0.0)
            {
                stage = 2;
            }

            // 
            // c     Test for warnings.
            // 
            if (brackt && (stp <= stmin || stp >= stmax))
            {
                task = Task.Warning;
            }
            if (brackt && stmax - stmin <= xtol * stmax)
            {
                task = Task.Warning;
            }
            if (stp == stpmax && f <= ftest && g <= gtest)
            {
                task = Task.Warning;
            }
            if (stp == stpmin && (f > ftest || g >= gtest))
            {
                task = Task.Warning;
            }

            // 
            // c     Test for convergence.
            // 
            if (f <= ftest && System.Math.Abs(g) <= gtol * -ginit)
            {
                task = Task.Convergence;
            }
            // 
            // c     Test for termination.
            // 
            if (task == Task.Warning || task == Task.Convergence)
            {
                goto L1000;
            }

            // 
            // c     A modified function is used to predict the step during the
            // c     first stage if a lower function value has been obtained but 
            // c     the decrease is not sufficient.
            // 
            if (stage == 1 && f <= fx && f > ftest)
            {
                // 
                // c        Define the modified function and derivative values.
                // 
                fm = f - stp * gtest;
                fxm = fx - stx * gtest;
                fym = fy - sty * gtest;
                gm = g - gtest;
                gxm = gx - gtest;
                gym = gy - gtest;

                // 
                // Call dcstep to update stx, sty, and to compute the new step.
                // 
                Dcstep(ref stx, ref fxm, ref gxm, ref sty, ref fym, ref gym,
                    ref stp, fm, gm, ref brackt, stmin, stmax);

                // 
                // Reset the function and derivative values for f.
                // 
                fx = fxm + stx * gtest;
                fy = fym + sty * gtest;
                gx = gxm + gtest;
                gy = gym + gtest;
                // 
            }
            else
            {
                // 
                // Call dcstep to update stx, sty, and to compute the new step.
                // 
                Dcstep(ref stx, ref fx, ref gx, ref sty, ref fy, ref gy,
                    ref stp, f, g, ref brackt, stmin, stmax);
            }

            // 
            // c     Decide if a bisection step is needed.
            // 
            if (brackt)
            {
                if (System.Math.Abs(sty - stx) >= 0.6600000000000000310862446895043831318617 * width1)
                {
                    stp = stx + 0.5 * (sty - stx);
                }
                width1 = width;
                width = System.Math.Abs(sty - stx);
            }
            // 
            // c     Set the minimum and maximum steps allowed for stp.
            // 
            if (brackt)
            {
                stmin = System.Math.Min(stx, sty);
                stmax = System.Math.Max(stx, sty);
            }
            else
            {
                stmin = stp + 1.100000000000000088817841970012523233891 * (stp - stx);
                stmax = stp + 4.0 * (stp - stx);
            }

            // 
            // c     Force the step to be within the bounds stpmax and stpmin.
            // 
            stp = System.Math.Max(stp, stpmin);
            stp = System.Math.Min(stp, stpmax);
            // 
            // c     If further progress is not possible, let stp be the best
            // c     point obtained during the search.
            // 
            if ((brackt && (stp <= stmin || stp >= stmax)) || (brackt && stmax - stmin <= xtol * stmax))
            {
                stp = stx;
            }
            // 
            // c     Obtain another function and derivative.
            // 
            task = Task.Fg;

        // 
        L1000:

            // 
            // c     Save local variables.
            // 
            if (brackt)
            {
                isave[1 - 1 + isaveOffset] = 1;
            }
            else
            {
                isave[1 - 1 + isaveOffset] = 0;
            }

            isave[2 - 1 + isaveOffset] = stage;
            dsave[1 - 1 + dsaveOffset] = ginit;
            dsave[2 - 1 + dsaveOffset] = gtest;
            dsave[3 - 1 + dsaveOffset] = gx;
            dsave[4 - 1 + dsaveOffset] = gy;
            dsave[5 - 1 + dsaveOffset] = finit;
            dsave[6 - 1 + dsaveOffset] = fx;
            dsave[7 - 1 + dsaveOffset] = fy;
            dsave[8 - 1 + dsaveOffset] = stx;
            dsave[9 - 1 + dsaveOffset] = sty;
            dsave[10 - 1 + dsaveOffset] = stmin;
            dsave[11 - 1 + dsaveOffset] = stmax;
            dsave[12 - 1 + dsaveOffset] = width;
            dsave[13 - 1 + dsaveOffset] = width1;
        }

        // 
        // c====================== The end of dcsrch ==============================
        // 
        // c     **********
        // c
        // c     Subroutine dcstep
        // c
        // c     This subroutine computes a safeguarded step for a search
        // c     procedure and updates an interval that contains a step that
        // c     satisfies a sufficient decrease and a curvature condition.
        // c
        // c     The parameter stx contains the step with the least function
        // c     value. If brackt is set to .true. then a minimizer has
        // c     been bracketed in an interval with endpoints stx and sty.
        // c     The parameter stp contains the current step. 
        // c     The subroutine assumes that if brackt is set to .true. then
        // c
        // c           min(stx,sty) < stp < max(stx,sty),
        // c
        // c     and that the derivative at stx is negative in the direction 
        // c     of the step.
        // c
        // c     The subroutine statement is
        // c
        // c       subroutine dcstep(stx,fx,dx,sty,fy,dy,stp,fp,dp,brackt,
        // c                         stpmin,stpmax)
        // c
        // c     where
        // c
        // c       stx is a double precision variable.
        // c         On entry stx is the best step obtained so far and is an
        // c            endpoint of the interval that contains the minimizer. 
        // c         On exit stx is the updated best step.
        // c
        // c       fx is a double precision variable.
        // c         On entry fx is the function at stx.
        // c         On exit fx is the function at stx.
        // c
        // c       dx is a double precision variable.
        // c         On entry dx is the derivative of the function at 
        // c            stx. The derivative must be negative in the direction of 
        // c            the step, that is, dx and stp - stx must have opposite 
        // c            signs.
        // c         On exit dx is the derivative of the function at stx.
        // c
        // c       sty is a double precision variable.
        // c         On entry sty is the second endpoint of the interval that 
        // c            contains the minimizer.
        // c         On exit sty is the updated endpoint of the interval that 
        // c            contains the minimizer.
        // c
        // c       fy is a double precision variable.
        // c         On entry fy is the function at sty.
        // c         On exit fy is the function at sty.
        // c
        // c       dy is a double precision variable.
        // c         On entry dy is the derivative of the function at sty.
        // c         On exit dy is the derivative of the function at the exit sty.

        // c
        // c       stp is a double precision variable.
        // c         On entry stp is the current step. If brackt is set to .true.
        // c            then on input stp must be between stx and sty. 
        // c         On exit stp is a new trial step.
        // c
        // c       fp is a double precision variable.
        // c         On entry fp is the function at stp
        // c         On exit fp is unchanged.
        // c
        // c       dp is a double precision variable.
        // c         On entry dp is the the derivative of the function at stp.
        // c         On exit dp is unchanged.
        // c
        // c       brackt is an logical variable.
        // c         On entry brackt specifies if a minimizer has been bracketed.
        // c            Initially brackt must be set to .false.
        // c         On exit brackt specifies if a minimizer has been bracketed.
        // c            When a minimizer is bracketed brackt is set to .true.
        // c
        // c       stpmin is a double precision variable.
        // c         On entry stpmin is a lower bound for the step.
        // c         On exit stpmin is unchanged.
        // c
        // c       stpmax is a double precision variable.
        // c         On entry stpmax is an upper bound for the step.
        // c         On exit stpmax is unchanged.
        // c
        // c     MINPACK-1 Project. June 1983
        // c     Argonne National Laboratory. 
        // c     Jorge J. More' and David J. Thuente.
        // c
        // c     MINPACK-2 Project. October 1993.
        // c     Argonne National Laboratory and University of Minnesota. 
        // c     Brett M. Averick and Jorge J. More'.
        // c
        // c     **********
        // 
        // 
        internal static void Dcstep(ref double stx, ref double fx, ref double dx,
            ref double sty, ref double fy, ref double dy, ref double stp,
            double fp, double dp, ref bool brackt, double stpmin, double stpmax)
        {
            double gamma = 0.0d;
            double p = 0.0d;
            double q = 0.0d;
            double r = 0.0d;
            double s = 0.0d;
            double sgnd = 0.0d;
            double stpc = 0.0d;
            double stpf = 0.0d;
            double stpq = 0.0d;
            double theta = 0.0d;
            sgnd = dp * (dx / System.Math.Abs(dx));

            // c     First case: A higher function value. The minimum is bracketed. 
            // c     If the cubic step is closer to stx than the quadratic step, the 
            // c     cubic step is taken, otherwise the average of the cubic and 
            // c     quadratic steps is taken.

            if (fp > fx)
            {
                theta = 3.0 * (fx - fp) / (stp - stx) + dx + dp;

                s = System.Math.Max(System.Math.Abs(theta), System.Math.Max(System.Math.Abs(dx), System.Math.Abs(dp)));

                gamma = s * System.Math.Sqrt(System.Math.Pow(theta / s, 2) - dx / s * (dp / s));
                if (stp < stx)
                {
                    gamma = -gamma;
                }
                p = gamma - dx + theta;
                q = gamma - dx + gamma + dp;
                r = p / q;
                stpc = stx + r * (stp - stx);
                stpq = stx + dx / ((fx - fp) / (stp - stx) + dx) / 2.0 * (stp - stx);

                if (System.Math.Abs(stpc - stx) < System.Math.Abs(stpq - stx))
                {
                    stpf = stpc;
                }
                else
                {
                    stpf = stpc + (stpq - stpc) / 2.0;
                }

                brackt = true;

                // 
                // c     Second case: A lower function value and derivatives of opposite 
                // c     sign. The minimum is bracketed. If the cubic step is farther from 
                // c     stp than the secant step, the cubic step is taken, otherwise the 

                // c     secant step is taken.
                // 
            }
            else if (sgnd < 0.0)
            {
                theta = 3.0 * (fx - fp) / (stp - stx) + dx + dp;
                s = System.Math.Max(System.Math.Abs(theta), System.Math.Max(System.Math.Abs(dx), System.Math.Abs(dp)));
                gamma = s * System.Math.Sqrt(System.Math.Pow(theta / s, 2) - dx / s * (dp / s));

                if (stp > stx)
                {
                    gamma = -gamma;
                }

                p = gamma - dp + theta;
                q = gamma - dp + gamma + dx;
                r = p / q;
                stpc = stp + r * (stx - stp);
                stpq = stp + dp / (dp - dx) * (stx - stp);

                if (System.Math.Abs(stpc - stp) > System.Math.Abs(stpq - stp))
                {
                    stpf = stpc;
                }
                else
                {
                    stpf = stpq;
                }

                brackt = true;
                // 
                // c     Third case: A lower function value, derivatives of the same sign,

                // c     and the magnitude of the derivative decreases.
                // 
            }
            else if (System.Math.Abs(dp) < System.Math.Abs(dx))
            {
                // 
                // c        The cubic step is computed only if the cubic tends to infinity 
                // c        in the direction of the step or if the minimum of the cubic
                // c        is beyond stp. Otherwise the cubic step is defined to be the 
                // c        secant step.
                // 
                theta = 3.0 * (fx - fp) / (stp - stx) + dx + dp;
                s = System.Math.Max(System.Math.Abs(theta), System.Math.Max(System.Math.Abs(dx), System.Math.Abs(dp)));

                // 
                // c        The case gamma = 0 only arises if the cubic does not tend
                // c        to infinity in the direction of the step.
                // 
                gamma = s * System.Math.Sqrt(System.Math.Max(0.0,
                    System.Math.Pow(theta / s, 2) - dx / s * (dp / s)));

                if (stp > stx)
                {
                    gamma = -gamma;
                }

                p = gamma - dp + theta;
                q = gamma + (dx - dp) + gamma;
                r = p / q;

                if (r < 0.0 && gamma != 0.0)
                {
                    stpc = stp + r * (stx - stp);
                }
                else if (stp > stx)
                {
                    stpc = stpmax;
                }
                else
                {
                    stpc = stpmin;
                }

                stpq = stp + dp / (dp - dx) * (stx - stp);

                if (brackt)
                {
                    // 
                    // c           A minimizer has been bracketed. If the cubic step is 
                    // c           closer to stp than the secant step, the cubic step is 
                    // c           taken, otherwise the secant step is taken.
                    // 
                    if (System.Math.Abs(stpc - stp) < System.Math.Abs(stpq - stp))
                    {
                        stpf = stpc;
                    }
                    else
                    {
                        stpf = stpq;
                    }
                    if (stp > stx)
                    {
                        stpf = System.Math.Min(stp +
                                               0.6600000000000000310862446895043831318617 * (sty - stp), stpf);
                    }
                    else
                    {
                        stpf = System.Math.Max(stp +
                                               0.6600000000000000310862446895043831318617 * (sty - stp), stpf);
                    }
                }
                else
                {
                    // 
                    // c           A minimizer has not been bracketed. If the cubic step is 
                    // c           farther from stp than the secant step, the cubic step is 
                    // c           taken, otherwise the secant step is taken.
                    // 
                    if (System.Math.Abs(stpc - stp) > System.Math.Abs(stpq - stp))
                    {
                        stpf = stpc;
                    }
                    else
                    {
                        stpf = stpq;
                    }
                    stpf = System.Math.Min(stpmax, stpf);
                    stpf = System.Math.Max(stpmin, stpf);
                }

                // 
                // c     Fourth case: A lower function value, derivatives of the same sign,
                // c     and the magnitude of the derivative does not decrease. If the 
                // c     minimum is not bracketed, the step is either stpmin or stpmax, 
                // c     otherwise the cubic step is taken.
                // 
            }
            else
            {
                if (brackt)
                {
                    theta = 3.0 * (fp - fy) / (sty - stp) + dy + dp;
                    s = System.Math.Max(System.Math.Abs(theta), System.Math.Max(System.Math.Abs(dy), System.Math.Abs(dp)));
                    gamma = s * System.Math.Sqrt(System.Math.Pow(theta / s, 2) - dy / s * (dp / s));

                    if (stp > sty)
                    {
                        gamma = -gamma;
                    }

                    p = gamma - dp + theta;
                    q = gamma - dp + gamma + dy;
                    r = p / q;
                    stpc = stp + r * (sty - stp);
                    stpf = stpc;
                }
                else if (stp > stx)
                {
                    stpf = stpmax;
                }
                else
                {
                    stpf = stpmin;
                }
            }

            // 
            // c     Update the interval which contains a minimizer.
            // 
            if (fp > fx)
            {
                sty = stp;
                fy = fp;
                dy = dp;
            }
            else
            {
                if (sgnd < 0.0)
                {
                    sty = stx;
                    fy = fx;
                    dy = dx;
                }
                stx = stp;
                fx = fp;
                dx = dp;
            }

            // 
            // c     Compute the new step.
            // 
            stp = stpf;
        }

        // 
        // c======================= The end of cmprlb =============================
        // 
        // 
        // 
        // c     ************
        // c
        // c     Subroutine errclb
        // c
        // c     This subroutine checks the validity of the input data.
        // c
        // c
        // c                           *  *  *
        // c
        // c     NEOS, November 1994. (Latest revision June 1996.)
        // c     Optimization Technology Center.
        // c     Argonne National Laboratory and Northwestern University.
        // c     Written by
        // c                        Ciyou Zhu
        // c     in collaboration with R.H. Byrd, P. Lu-Chen and J. Nocedal.
        // c
        // c
        // c     ************
        // 
        // 
        // c     Check the input arguments for errors.
        // 
        private static void Errclb(int n, int m, double factr,
            double[] l, int lOffset, double[] u, int uOffset,
            int[] nbd, int nbdOffset, ref Task task, ref int info, ref int k)
        {

            int i = 0;
            if (n <= 0)
            {
                task = Task.Error;
            }
            if (m <= 0)
            {
                task = Task.Error;
            }
            if (factr < 0.0)
            {
                task = Task.Error;
            }

            // 
            // c     Check the validity of the arrays nbd(i), u(i), and l(i).
            // 
            {
                for (i = 1; i <= n; i++)
                {
                    if (nbd[i - 1 + nbdOffset] < 0 || nbd[i - 1 + nbdOffset] > 3)
                    {
                        // c                                                   return
                        task = Task.Error;
                        info = -6;
                        k = i;
                    }
                    if (nbd[i - 1 + nbdOffset] == 2)
                    {
                        if (l[i - 1 + lOffset] > u[i - 1 + uOffset])
                        {
                            // c                                    return
                            task = Task.Error;
                            info = -7;
                            k = i;
                        }
                    }
                }
            }

        }

        // 
        // c======================= The end of errclb =============================
        // 
        // 
        // 
        // c     ************
        // c
        // c     Subroutine formk 
        // c
        // c     This subroutine forms  the LEL^T factorization of the indefinite
        // c
        // c       matrix    K = [-D -Y'ZZ'Y/theta     L_a'-R_z'  ]
        // c                     [L_a -R_z           theta*S'AA'S ]
        // c                                                    where E = [-I  0]
        // c                                                              [ 0  I]
        // c     The matrix K can be shown to be equal to the matrix M^[-1]N
        // c       occurring in section 5.1 of [1], as well as to the matrix
        // c       Mbar^[-1] Nbar in section 5.3.
        // c
        // c     n is an integer variable.
        // c       On entry n is the dimension of the problem.
        // c       On exit n is unchanged.
        // c
        // c     nsub is an integer variable
        // c       On entry nsub is the number of subspace variables in free set.
        // c       On exit nsub is not changed.
        // c
        // c     ind is an integer array of dimension nsub.
        // c       On entry ind specifies the indices of subspace variables.
        // c       On exit ind is unchanged. 
        // c
        // c     nenter is an integer variable.
        // c       On entry nenter is the number of variables entering the 
        // c         free set.
        // c       On exit nenter is unchanged. 
        // c
        // c     ileave is an integer variable.
        // c       On entry indx2(ileave),...,indx2(n) are the variables leaving
        // c         the free set.
        // c       On exit ileave is unchanged. 
        // c
        // c     indx2 is an integer array of dimension n.
        // c       On entry indx2(1),...,indx2(nenter) are the variables entering
        // c         the free set, while indx2(ileave),...,indx2(n) are the
        // c         variables leaving the free set.
        // c       On exit indx2 is unchanged. 
        // c
        // c     iupdat is an integer variable.
        // c       On entry iupdat is the total number of BFGS updates made so far.
        // c       On exit iupdat is unchanged. 
        // c
        // c     updatd is a logical variable.
        // c       On entry 'updatd' is true if the L-BFGS matrix is updatd.
        // c       On exit 'updatd' is unchanged. 
        // c
        // c     wn is a double precision array of dimension 2m x 2m.
        // c       On entry wn is unspecified.
        // c       On exit the upper triangle of wn stores the LEL^T factorization

        // c         of the 2*col x 2*col indefinite matrix
        // c                     [-D -Y'ZZ'Y/theta     L_a'-R_z'  ]
        // c                     [L_a -R_z           theta*S'AA'S ]
        // c
        // c     wn1 is a double precision array of dimension 2m x 2m.
        // c       On entry wn1 stores the lower triangular part of 
        // c                     [Y' ZZ'Y   L_a'+R_z']
        // c                     [L_a+R_z   S'AA'S   ]
        // c         in the previous iteration.
        // c       On exit wn1 stores the corresponding updated matrices.
        // c       The purpose of wn1 is just to store these inner products
        // c       so they can be easily updated and inserted into wn.
        // c
        // c     m is an integer variable.
        // c       On entry m is the maximum number of variable metric corrections

        // c         used to define the limited memory matrix.
        // c       On exit m is unchanged.
        // c
        // c     ws, wy, sy, and wtyy are double precision arrays;
        // c     theta is a double precision variable;
        // c     col is an integer variable;
        // c     head is an integer variable.
        // c       On entry they store the information defining the
        // c                                          limited memory BFGS matrix:
        // c         ws(n,m) stores S, a set of s-vectors;
        // c         wy(n,m) stores Y, a set of y-vectors;
        // c         sy(m,m) stores S'Y;
        // c         wtyy(m,m) stores the Cholesky factorization
        // c                                   of (theta*S'S+LD^(-1)L')
        // c         theta is the scaling factor specifying B_0 = theta I;
        // c         col is the number of variable metric corrections stored;
        // c         head is the location of the 1st s- (or y-) vector in S (or Y).
        // c       On exit they are unchanged.
        // c
        // c     info is an integer variable.
        // c       On entry info is unspecified.
        // c       On exit info =  0 for normal return;
        // c                    = -1 when the 1st Cholesky factorization failed;
        // c                    = -2 when the 2st Cholesky factorization failed.
        // c
        // c     Subprograms called:
        // c
        // c       Linpack ... dcopy, dpofa, 
        // c
        // c
        // c     References:
        // c       [1] R. H. Byrd, P. Lu, J. Nocedal and C. Zhu, ``A limited
        // c       memory algorithm for bound constrained optimization'',
        // c       SIAM J. Scientific Computing 16 (1995), no. 5, pp. 1190--1208.
        // c
        // c       [2] C. Zhu, R.H. Byrd, P. Lu, J. Nocedal, ``L-BFGS-B: a
        // c       limited memory FORTRAN code for solving bound constrained
        // c       optimization problems'', Tech. Report, NAM-11, EECS Department,

        // c       Northwestern University, 1994.
        // c
        // c       (Postscript files of these papers are available via anonymous
        // c        ftp to eecs.nwu.edu in the directory pub/lbfgs/lbfgs_bcm.)
        // c
        // c                           *  *  *
        // c
        // c     NEOS, November 1994. (Latest revision June 1996.)
        // c     Optimization Technology Center.
        // c     Argonne National Laboratory and Northwestern University.
        // c     Written by
        // c                        Ciyou Zhu
        // c     in collaboration with R.H. Byrd, P. Lu-Chen and J. Nocedal.
        // c
        // c
        // c     ************
        // 
        // 
        // c     Form the lower triangular part of
        // c               WN1 = [Y' ZZ'Y   L_a'+R_z'] 
        // c                     [L_a+R_z   S'AA'S   ]
        // c        where L_a is the strictly lower triangular part of S'AA'Y
        // c              R_z is the upper triangular part of S'ZZ'Y.
        // 
        private static void Formk(int n, int nsub, int[] ind, int indOffset,
            int nenter, int ileave, int[] indx2, int indx2Offset,
            int iupdat, bool updatd, double[] wn, int wnOffset, double[] wn1, int wn1Offset,
            int m, double[] ws, int wsOffset, double[] wy, int wyOffset,
            double[] sy, int syOffset, double theta, int col, int head,
            ref int info)
        {

            int m2 = 0;
            int ipntr = 0;
            int jpntr = 0;
            int iy = 0;
            int is2 = 0;
            int jy = 0;
            int js = 0;
            int is1 = 0;
            int js1 = 0;
            int k1 = 0;
            int i = 0;
            int k = 0;
            int col2 = 0;
            int pbegin = 0;
            int pend = 0;
            int dbegin = 0;
            int dend = 0;
            int upcl = 0;
            double temp1 = 0.0d;
            double temp2 = 0.0d;
            double temp3 = 0.0d;
            double temp4 = 0.0d;

            if (updatd)
            {
                if (iupdat > m)
                {
                    // c                                 shift old part of WN1.
                    {
                        for (jy = 1; jy <= m - 1; jy++)
                        {
                            js = m + jy;
                            Dcopy(m - jy, wn1, jy + 1 - 1 + (jy + 1 - 1) * 2 * m
                                                          + wn1Offset, 1, wn1, jy - 1 + (jy - 1) * 2 * m + wn1Offset, 1);
                            Dcopy(m - jy, wn1, js + 1 - 1 + (js + 1 - 1) * 2 * m
                                                          + wn1Offset, 1, wn1, js - 1 + (js - 1) * 2 * m + wn1Offset, 1);
                            Dcopy(m - 1, wn1, m + 2 - 1 + (jy + 1 - 1) * 2 * m
                                                        + wn1Offset, 1, wn1, m + 1 - 1 + (jy - 1) * 2 * m + wn1Offset, 1);
                        }
                    }
                }

                // 
                // c          put new rows in blocks (1,1), (2,1) and (2,2).
                pbegin = 1;
                pend = nsub;
                dbegin = nsub + 1;
                dend = n;
                iy = col;
                is2 = m + col;
                ipntr = head + col - 1;
                if (ipntr > m)
                {
                    ipntr -= m;
                }
                jpntr = head;
                {
                    for (jy = 1; jy <= col; jy++)
                    {
                        js = m + jy;
                        temp1 = 0.0;
                        temp2 = 0.0;
                        temp3 = 0.0;

                        // c             compute element jy of row 'col' of Y'ZZ'Y
                        {
                            for (k = pbegin; k <= pend; k++)
                            {
                                k1 = ind[k - 1 + indOffset];
                                temp1 += wy[k1 - 1 + (ipntr - 1)
                                    * n + wyOffset] * wy[k1 - 1 + (jpntr - 1) * n + wyOffset];
                            }
                        }
                        // c             compute elements jy of row 'col' of L_a and S'AA'S
                        {
                            for (k = dbegin; k <= dend; k++)
                            {
                                k1 = ind[k - 1 + indOffset];
                                temp2 += ws[k1 - 1 + (ipntr - 1)
                                    * n + wsOffset] * ws[k1 - 1 + (jpntr - 1) * n + wsOffset];
                                temp3 += ws[k1 - 1 + (ipntr - 1)
                                    * n + wsOffset] * wy[k1 - 1 + (jpntr - 1) * n + wyOffset];
                            }
                        }

                        wn1[iy - 1 + (jy - 1) * 2 * m + wn1Offset] = temp1;
                        wn1[is2 - 1 + (js - 1) * 2 * m + wn1Offset] = temp2;
                        wn1[is2 - 1 + (jy - 1) * 2 * m + wn1Offset] = temp3;
                        jpntr = jpntr % m + 1;
                    }
                }

                // 
                // c          put new column in block (2,1).
                jy = col;
                jpntr = head + col - 1;
                if (jpntr > m)
                {
                    jpntr -= m;
                }
                ipntr = head;
                {
                    for (i = 1; i <= col; i++)
                    {
                        is2 = m + i;
                        temp3 = 0.0;
                        // c             compute element i of column 'col' of R_z
                        {
                            for (k = pbegin; k <= pend; k++)
                            {
                                k1 = ind[k - 1 + indOffset];
                                temp3 += ws[k1 - 1 + (ipntr - 1)
                                    * n + wsOffset] * wy[k1 - 1 + (jpntr - 1) * n + wyOffset];
                            }
                        }
                        ipntr = ipntr % m + 1;
                        wn1[is2 - 1 + (jy - 1) * 2 * m + wn1Offset] = temp3;
                    }
                }
                upcl = col - 1;
            }
            else
            {
                upcl = col;
            }

            // 
            // c       modify the old parts in blocks (1,1) and (2,2) due to changes
            // c       in the set of free variables.

            ipntr = head;
            {
                for (iy = 1; iy <= upcl; iy++)
                {
                    is2 = m + iy;
                    jpntr = head;
                    {
                        for (jy = 1; jy <= iy; jy++)
                        {
                            js = m + jy;
                            temp1 = 0.0;
                            temp2 = 0.0;
                            temp3 = 0.0;
                            temp4 = 0.0;
                            {
                                for (k = 1; k <= nenter; k++)
                                {
                                    k1 = indx2[k - 1 + indx2Offset];
                                    temp1 += wy[k1 - 1 + (ipntr - 1)
                                        * n + wyOffset] * wy[k1 - 1 + (jpntr - 1) * n + wyOffset];
                                    temp2 += ws[k1 - 1 + (ipntr - 1)
                                        * n + wsOffset] * ws[k1 - 1 + (jpntr - 1) * n + wsOffset];
                                }
                            }
                            {
                                for (k = ileave; k <= n; k++)
                                {
                                    k1 = indx2[k - 1 + indx2Offset];
                                    temp3 += wy[k1 - 1 + (ipntr
                                                          - 1) * n + wyOffset] * wy[k1 - 1 + (jpntr - 1) * n + wyOffset];
                                    temp4 += ws[k1 - 1 + (ipntr
                                                          - 1) * n + wsOffset] * ws[k1 - 1 + (jpntr - 1) * n + wsOffset];
                                }
                            }

                            wn1[iy - 1 + (jy - 1) * 2 * m + wn1Offset] =
                                wn1[iy - 1 + (jy - 1) * 2 * m + wn1Offset] + temp1 - temp3;
                            wn1[is2 - 1 + (js - 1) * 2 * m + wn1Offset] =
                                wn1[is2 - 1 + (js - 1) * 2 * m + wn1Offset] - temp2 + temp4;
                            jpntr = jpntr % m + 1;
                        }
                    }

                    ipntr = ipntr % m + 1;
                }
            }

            // 
            // c       modify the old parts in block (2,1).
            ipntr = head;
            {
                for (is2 = m + 1; is2 <= m + upcl; is2++)
                {
                    jpntr = head;
                    {
                        for (jy = 1; jy <= upcl; jy++)
                        {
                            temp1 = 0.0;
                            temp3 = 0.0;
                            {
                                for (k = 1; k <= nenter; k++)
                                {
                                    k1 = indx2[k - 1 + indx2Offset];
                                    temp1 += ws[k1 - 1 + (ipntr
                                                          - 1) * n + wsOffset] * wy[k1 - 1 + (jpntr - 1) * n + wyOffset];
                                }
                            }
                            {
                                for (k = ileave; k <= n; k++)
                                {
                                    k1 = indx2[k - 1 + indx2Offset];
                                    temp3 += ws[k1 - 1 + (ipntr - 1)
                                        * n + wsOffset] * wy[k1 - 1 + (jpntr - 1) * n + wyOffset];
                                }
                            }

                            if (is2 <= jy + m)
                            {
                                wn1[is2 - 1 + (jy - 1) * 2 * m + wn1Offset] =
                                    wn1[is2 - 1 + (jy - 1) * 2 * m + wn1Offset] + temp1 - temp3;
                            }
                            else
                            {
                                wn1[is2 - 1 + (jy - 1) * 2 * m + wn1Offset] =
                                    wn1[is2 - 1 + (jy - 1) * 2 * m + wn1Offset] - temp1 + temp3;
                            }
                            jpntr = jpntr % m + 1;
                        }
                    }
                    ipntr = ipntr % m + 1;
                }
            }

            // 
            // c     Form the upper triangle of WN = [D+Y' ZZ'Y/theta   -L_a'+R_z' ] 
            // c                                     [-L_a +R_z        S'AA'S*theta]
            // 
            m2 = 2 * m;
            {
                for (iy = 1; iy <= col; iy++)
                {
                    is2 = col + iy;
                    is1 = m + iy;
                    {
                        for (jy = 1; jy <= iy; jy++)
                        {
                            js = col + jy;
                            js1 = m + jy;
                            wn[jy - 1 + (iy - 1) * 2 * m + wnOffset] =
                                wn1[iy - 1 + (jy - 1) * 2 * m + wn1Offset] / theta;
                            wn[js - 1 + (is2 - 1) * 2 * m + wnOffset] =
                                wn1[is1 - 1 + (js1 - 1) * 2 * m + wn1Offset] * theta;
                        }
                    }
                    {
                        for (jy = 1; jy <= iy - 1; jy++)
                        {
                            wn[jy - 1 + (is2 - 1) * 2 * m + wnOffset] =
                                -wn1[is1 - 1 + (jy - 1) * 2 * m + wn1Offset];
                        }
                    }
                    {
                        for (jy = iy; jy <= col; jy++)
                        {
                            wn[jy - 1 + (is2 - 1) * 2 * m + wnOffset] =
                                wn1[is1 - 1 + (jy - 1) * 2 * m + wn1Offset];
                        }
                    }

                    wn[iy - 1 + (iy - 1) * 2 * m + wnOffset] += sy[iy - 1 + (iy - 1) * m + syOffset];
                }
            }

            // 
            // c     Form the upper triangle of WN= [  LL'            L^-1(-L_a'+R_z')]
            // c                                    [(-L_a +R_z)L'^-1   S'AA'S*theta  ]
            // 
            // c        first Cholesky factor (1,1) block of wn to get LL'
            // c                          with L' stored in the upper triangle of wn.
            Dpofa(wn, wnOffset, m2, col, ref info);

            // c        then form L^-1(-L_a'+R_z') in the (1,2) block.
            col2 = 2 * col;
            {
                for (js = col + 1; js <= col2; js++)
                {
                    Dtrsl(wn, wnOffset, m2, col, wn, 1 - 1
                        + (js - 1) * 2 * m + wnOffset, 11, out info);
                }
            }

            // 
            // c     Form S'AA'S*theta + (L^-1(-L_a'+R_z'))'L^-1(-L_a'+R_z') in the
            // c        upper triangle of (2,2) block of wn.
            // 
            // 
            {
                for (is2 = col + 1; is2 <= col2; is2++)
                {
                    {
                        for (js = is2; js <= col2; js++)
                        {
                            wn[is2 - 1 + (js - 1) * 2 * m + wnOffset] += Ddot(col, wn, 1 - 1 + (is2 - 1) * 2 * m
                                + wnOffset, 1, wn, 1 - 1 + (js - 1) * 2 * m + wnOffset, 1);
                        }
                    }
                }
            }

            // 
            // c     Cholesky factorization of (2,2) block of wn.
            // 
            Dpofa(wn, col + 1 - 1 + (col + 1 - 1) 
                * 2 * m + wnOffset, m2, col, ref info);
        }

        // 
        // c======================= The end of formk ==============================
        // 
        // 
        // 
        // c     ************
        // c
        // c     Subroutine formt
        // c
        // c       This subroutine forms the upper half of the pos. def. and symm.

        // c         T = theta*SS + L*D^(-1)*L', stores T in the upper triangle
        // c         of the array wt, and performs the Cholesky factorization of T

        // c         to produce J*J', with J' stored in the upper triangle of wt.
        // c
        // c     Subprograms called:
        // c
        // c       Linpack ... dpofa.
        // c
        // c
        // c                           *  *  *
        // c
        // c     NEOS, November 1994. (Latest revision June 1996.)
        // c     Optimization Technology Center.
        // c     Argonne National Laboratory and Northwestern University.
        // c     Written by
        // c                        Ciyou Zhu
        // c     in collaboration with R.H. Byrd, P. Lu-Chen and J. Nocedal.
        // c
        // c
        // c     ************
        // 
        // 
        // 
        // c     Form the upper half of  T = theta*SS + L*D^(-1)*L',
        // c        store T in the upper triangle of the array wt.
        // 
        private static void Formt(int m,
            double[] wt, int wtOffset,
            double[] sy, int syOffset,
            double[] ss, int ssOffset,
            int col, double theta, ref int info)
        {

            int i = 0;
            int j = 0;
            int k = 0;
            int k1 = 0;

            double ddum = 0.0d;
            {
                for (j = 1; j <= col; j++)
                {
                    wt[1 - 1 + (j - 1) * m + wtOffset] = theta * ss[1 - 1
                                                                      + (j - 1) * m + ssOffset];
                }
            }

            {
                for (i = 2; i <= col; i++)
                {
                    {
                        for (j = i; j <= col; j++)
                        {
                            k1 = System.Math.Min(i, j) - 1;

                            ddum = 0.0;
                            {
                                for (k = 1; k <= k1; k++)
                                {
                                    ddum += sy[i - 1 + (k - 1) * m
                                                     + syOffset] * sy[j - 1 + (k - 1) * m
                                                                            + syOffset] / sy[k - 1 + (k - 1) * m + syOffset];
                                }
                            }

                            wt[i - 1 + (j - 1) * m + wtOffset] = ddum
                                                                   + theta * ss[i - 1 + (j - 1) * m + ssOffset];
                        }
                    }
                }
            }

            // 
            // c     Cholesky factorize T to J*J' with 
            // c        J' stored in the upper triangle of wt.
            // 
            Dpofa(wt, wtOffset, m, col, ref info);
        }

        // 
        // c======================= The end of formt ==============================
        // 
        // 
        // 
        // c     ************
        // c
        // c     Subroutine freev 
        // c
        // c     This subroutine counts the entering and leaving variables when
        // c       iter > 0, and finds the index set of free and active variables
        // c       at the GCP.
        // c
        // c     cnstnd is a logical variable indicating whether bounds are present
        // c
        // c     index is an integer array of dimension n
        // c       for i=1,...,nfree, index(i) are the indices of free variables
        // c       for i=nfree+1,...,n, index(i) are the indices of bound variables
        // c       On entry after the first iteration, index gives 
        // c         the free variables at the previous iteration.
        // c       On exit it gives the free variables based on the determination
        // c         in cauchy using the array iwhere.
        // c
        // c     indx2 is an integer array of dimension n
        // c       On entry indx2 is unspecified.
        // c       On exit with iter>0, indx2 indicates which variables
        // c          have changed status since the previous iteration.
        // c       For i= 1,...,nenter, indx2(i) have changed from bound to free.
        // c       For i= ileave+1,...,n, indx2(i) have changed from free to bound.
        // c 
        // c
        // c                           *  *  *
        // c
        // c     NEOS, November 1994. (Latest revision June 1996.)
        // c     Optimization Technology Center.
        // c     Argonne National Laboratory and Northwestern University.
        // c     Written by
        // c                        Ciyou Zhu
        // c     in collaboration with R.H. Byrd, P. Lu-Chen and J. Nocedal.
        // c
        // c
        // c     ************
        // 
        // 
        private static void Freev(int n, ref int nfree,
            int[] index, int indexOffset, ref int nenter, ref int ileave,
            int[] indx2, int indx2Offset, int[] iwhere, int iwhereOffset,
            ref bool wrk, bool updatd, bool cnstnd, int iprint, int iter)
        {

            int iact = 0;
            int i = 0;
            int k = 0;

            nenter = 0;
            ileave = n + 1;

            if (iter > 0 && cnstnd)
            {
                // c                           count the entering and leaving variables.
                {
                    for (i = 1; i <= nfree; i++)
                    {
                        k = index[i - 1 + indexOffset];

                        // 
                        // c            write(6,*) ' k  = index(i) ', k
                        // c            write(6,*) ' index = ', i
                        // 
                        if (iwhere[k - 1 + iwhereOffset] > 0)
                        {
                            ileave -= 1;
                            indx2[ileave - 1 + indx2Offset] = k;

                            if (iprint >= 100)
                            {
                                // DISPLAY: "Variable " + k + " leaves the set of free variables"
                            }
                        }
                    }
                }
                {
                    for (i = 1 + nfree; i <= n; i++)
                    {
                        k = index[i - 1 + indexOffset];
                        if (iwhere[k - 1 + iwhereOffset] <= 0)
                        {
                            nenter += 1;
                            indx2[nenter - 1 + indx2Offset] = k;

                            
                            if (iprint >= 100)
                            {
                                // DISPLAY: "Variable " + k + " enters the set of free variables"
                            }
                        }
                    }
                }
                
                if (iprint >= 99)
                {
                    // DISPLAY: ((n + 1) - ileave)) + " variables leave; "
                    //           nenter + " variables enter"
                }
            }

            wrk = ileave < n + 1 || nenter > 0 || updatd;

            // 
            // c     Find the index set of free and active variables at the GCP.
            // 
            nfree = 0;
            iact = n + 1;
            {
                for (i = 1; i <= n; i++)
                {
                    if (iwhere[i - 1 + iwhereOffset] <= 0)
                    {
                        nfree += 1;
                        index[nfree - 1 + indexOffset] = i;
                    }
                    else
                    {
                        iact -= 1;
                        index[iact - 1 + indexOffset] = i;
                    }
                }
            }

            if (iprint >= 99)
            {
                // DISPLAY: nfree + " variables are free at GCP " + (iter + 1))
            }
        }

        // 
        // c======================= The end of freev ==============================
        // 
        // 
        // c     ************
        // c
        // c     Subroutine hpsolb 
        // c
        // c     This subroutine sorts out the least element of t, and puts the
        // c       remaining elements of t in a heap.
        // c 
        // c     n is an integer variable.
        // c       On entry n is the dimension of the arrays t and iorder.
        // c       On exit n is unchanged.
        // c
        // c     t is a double precision array of dimension n.
        // c       On entry t stores the elements to be sorted,
        // c       On exit t(n) stores the least elements of t, and t(1) to t(n-1)

        // c         stores the remaining elements in the form of a heap.
        // c
        // c     iorder is an integer array of dimension n.
        // c       On entry iorder(i) is the index of t(i).
        // c       On exit iorder(i) is still the index of t(i), but iorder may be

        // c         permuted in accordance with t.
        // c
        // c     iheap is an integer variable specifying the task.
        // c       On entry iheap should be set as follows:
        // c         iheap .eq. 0 if t(1) to t(n) is not in the form of a heap,
        // c         iheap .ne. 0 if otherwise.
        // c       On exit iheap is unchanged.
        // c
        // c
        // c     References:
        // c       Algorithm 232 of CACM (J. W. J. Williams): HEAPSORT.
        // c
        // c                           *  *  *
        // c
        // c     NEOS, November 1994. (Latest revision June 1996.)
        // c     Optimization Technology Center.
        // c     Argonne National Laboratory and Northwestern University.
        // c     Written by
        // c                        Ciyou Zhu
        // c     in collaboration with R.H. Byrd, P. Lu-Chen and J. Nocedal.
        // c
        // c     ************
        // 
        // 
        private static void Hpsolb(int n, double[] t, int tOffset,
        int[] iorder, int iorderOffset, int iheap)
        {

            int i = 0;
            int j = 0;
            int k = 0;
            int indxin = 0;
            int indxou = 0;
            double ddum = 0.0d;
            double out2 = 0.0d;

            if (iheap == 0)
            {
                // 
                // c        Rearrange the elements t(1) to t(n) to form a heap.
                // 
                {
                    for (k = 2; k <= n; k++)
                    {
                        ddum = t[k - 1 + tOffset];
                        indxin = iorder[k - 1 + iorderOffset];
                        // 
                        // c           Add ddum to the heap.
                        i = k;

                    L10:
                        if (i > 1)
                        {
                            j = i / 2;
                            if (ddum < t[j - 1 + tOffset])
                            {
                                t[i - 1 + tOffset] = t[j - 1 + tOffset];
                                iorder[i - 1 + iorderOffset] = iorder[j - 1 + iorderOffset];
                                i = j;
                                goto L10;
                            }
                        }

                        t[i - 1 + tOffset] = ddum;
                        iorder[i - 1 + iorderOffset] = indxin;
                    }
                }
            }
            // 
            // c     Assign to 'out' the value of t(1), the least member of the heap,
            // c        and rearrange the remaining members to form a heap as
            // c        elements 1 to n-1 of t.
            // 
            if (n > 1)
            {
                i = 1;
                out2 = t[1 - 1 + tOffset];
                indxou = iorder[1 - 1 + iorderOffset];
                ddum = t[n - 1 + tOffset];
                indxin = iorder[n - 1 + iorderOffset];

            // 
            // c        Restore the heap 
            L30:
                j = i + i;
                if (j <= n - 1)
                {
                    if (t[j + 1 - 1 + tOffset] < t[j - 1 + tOffset])
                    {
                        j += 1;
                    }
                    if (t[j - 1 + tOffset] < ddum)
                    {
                        t[i - 1 + tOffset] = t[j - 1 + tOffset];
                        iorder[i - 1 + iorderOffset] = iorder[j - 1 + iorderOffset];
                        i = j;
                        goto L30;
                    }
                }
                t[i - 1 + tOffset] = ddum;
                iorder[i - 1 + iorderOffset] = indxin;
                // 
                // c     Put the least member in t(n). 
                // 
                t[n - 1 + tOffset] = out2;
                iorder[n - 1 + iorderOffset] = indxou;
            }
        }

        // 
        // c====================== The end of hpsolb ==============================
        // 
        // 
        // c     **********
        // c
        // c     Subroutine lnsrlb
        // c
        // c     This subroutine calls subroutine dcsrch from the Minpack2 library

        // c       to perform the line search.  Subroutine dscrch is safeguarded so
        // c       that all trial points lie within the feasible region.
        // c
        // c     Subprograms called:
        // c
        // c       Minpack2 Library ... dcsrch.
        // c
        // c       Linpack ... dtrsl, 
        // c
        // c
        // c                           *  *  *
        // c
        // c     NEOS, November 1994. (Latest revision June 1996.)
        // c     Optimization Technology Center.
        // c     Argonne National Laboratory and Northwestern University.
        // c     Written by
        // c                        Ciyou Zhu
        // c     in collaboration with R.H. Byrd, P. Lu-Chen and J. Nocedal.
        // c
        // c
        // c     **********
        // 
        // 
        private static void Lnsrlb(int n, double[] l, int lOffset, double[] u, int uOffset,
            int[] nbd, int nbdOffset, double[] x, int xOffset, double f, ref double fold, ref double gd,
            ref double gdold, double[] g, int gOffset, double[] d, int dOffset, double[] r, int rOffset,
            double[] t, int tOffset, double[] z, int zOffset, ref double stp,
            ref double dnorm, ref double dtd, ref double xstep, ref double stpmx, int iter,
            ref int ifun, ref int iback, ref int nfgv, ref int info, ref Task task,
            bool boxed, bool cnstnd, ref Task csave, int[] isave, int isaveOffset,
            double[] dsave, int dsaveOffset)
        {

            int i = 0;
            double a1 = 0.0d;
            double a2 = 0.0d;

            if (task == Task.FgLN)
            {
                goto L556;
            }

            // 
            dtd = Ddot(n, d, dOffset, 1, d, dOffset, 1);
            dnorm = System.Math.Sqrt(dtd);
            // 
            // c     Determine the maximum step length.
            // 
            stpmx = 10000000000.0;
            if (cnstnd)
            {
                if (iter == 0)
                {
                    stpmx = 1.0;
                }
                else
                {
                    {
                        for (i = 1; i <= n; i++)
                        {
                            a1 = d[i - 1 + dOffset];
                            if (nbd[i - 1 + nbdOffset] != 0)
                            {
                                if (a1 < 0.0 && nbd[i - 1 + nbdOffset] <= 2)
                                {
                                    a2 = l[i - 1 + lOffset] - x[i - 1 + xOffset];
                                    if (a2 >= 0.0)
                                    {
                                        stpmx = 0.0;
                                    }
                                    else if (a1 * stpmx < a2)
                                    {
                                        stpmx = a2 / a1;
                                    }
                                }
                                else if (a1 > 0.0 && nbd[i - 1 + nbdOffset] >= 2)
                                {
                                    a2 = u[i - 1 + uOffset] - x[i - 1 + xOffset];
                                    if (a2 <= 0.0)
                                    {
                                        stpmx = 0.0;
                                    }
                                    else if (a1 * stpmx > a2)
                                    {
                                        stpmx = a2 / a1;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // 
            if (iter == 0 && !boxed)
            {
                if (double.IsNaN(dnorm))
                    stp = stpmx;
                else
                    stp = System.Math.Min(1.0 / dnorm, stpmx);
            }
            else
            {
                stp = 1.0;
            }

            // 
            Dcopy(n, x, xOffset, 1, t, tOffset, 1);
            Dcopy(n, g, gOffset, 1, r, rOffset, 1);

            fold = f;
            ifun = 0;
            iback = 0;
            csave = Task.Start;

        L556:
            gd = Ddot(n, g, gOffset, 1, d, dOffset, 1);

            if (ifun == 0)
            {
                gdold = gd;
                if (gd >= 0.0)
                {
                    // the directional derivative >=0.
                    // Line search is impossible.

                    // DISPLAY: " ascent direction in projection gd = " + gd
                    info = -4;
                    return;
                }
            }

            Dcsrch(f, gd, ref stp,
                0.001000000000000000020816681711721685132943,
                0.9000000000000000222044604925031308084726,
                0.1000000000000000055511151231257827021182, 0.0,
                stpmx, ref csave, isave, isaveOffset, dsave, dsaveOffset);

            // 
            xstep = stp * dnorm;
            if (csave != Task.Convergence && csave != Task.Warning)
            {
                task = Task.FgLN;
                ifun += 1;
                nfgv += 1;
                iback = ifun - 1;
                if (stp == 1.0)
                {
                    Dcopy(n, z, zOffset, 1, x, xOffset, 1);
                }
                else
                {
                    {
                        for (i = 1; i <= n; i++)
                        {
                            x[i - 1 + xOffset] = stp * d[i - 1 + dOffset] + t[i - 1 + tOffset];
                        }
                    }
                }
            }
            else
            {
                task = Task.NewX;
            }
        }

        // 
        // c======================= The end of setulb =============================
        // 
        // c-jlm-jn
        // 
        // 
        // c     ************
        // c
        // c     Subroutine mainlb
        // c
        // c     This subroutine solves bound constrained optimization problems by

        // c       using the compact formula of the limited memory BFGS updates.
        // c       
        // c     n is an integer variable.
        // c       On entry n is the number of variables.
        // c       On exit n is unchanged.
        // c
        // c     m is an integer variable.
        // c       On entry m is the maximum number of variable metric
        // c          corrections allowed in the limited memory matrix.
        // c       On exit m is unchanged.
        // c
        // c     x is a double precision array of dimension n.
        // c       On entry x is an approximation to the solution.
        // c       On exit x is the current approximation.
        // c
        // c     l is a double precision array of dimension n.
        // c       On entry l is the lower bound of x.
        // c       On exit l is unchanged.
        // c
        // c     u is a double precision array of dimension n.
        // c       On entry u is the upper bound of x.
        // c       On exit u is unchanged.
        // c
        // c     nbd is an integer array of dimension n.
        // c       On entry nbd represents the type of bounds imposed on the
        // c         variables, and must be specified as follows:
        // c         nbd(i)=0 if x(i) is unbounded,
        // c                1 if x(i) has only a lower bound,
        // c                2 if x(i) has both lower and upper bounds,
        // c                3 if x(i) has only an upper bound.
        // c       On exit nbd is unchanged.
        // c
        // c     f is a double precision variable.
        // c       On first entry f is unspecified.
        // c       On final exit f is the value of the function at x.
        // c
        // c     g is a double precision array of dimension n.
        // c       On first entry g is unspecified.
        // c       On final exit g is the value of the gradient at x.
        // c
        // c     factr is a double precision variable.
        // c       On entry factr >= 0 is specified by the user.  The iteration
        // c         will stop when
        // c
        // c         (f^k - f^{k+1})/max{|f^k|,|f^{k+1}|,1} <= factr*epsmch
        // c
        // c         where epsmch is the machine precision, which is automatically

        // c         generated by the code.
        // c       On exit factr is unchanged.
        // c
        // c     pgtol is a double precision variable.
        // c       On entry pgtol >= 0 is specified by the user.  The iteration
        // c         will stop when
        // c
        // c                 max{|proj g_i | i = 1, ..., n} <= pgtol
        // c
        // c         where pg_i is the ith component of the projected gradient.
        // c       On exit pgtol is unchanged.
        // c
        // c     ws, wy, sy, and wt are double precision working arrays used to
        // c       store the following information defining the limited memory
        // c          BFGS matrix:
        // c          ws, of dimension n x m, stores S, the matrix of s-vectors;
        // c          wy, of dimension n x m, stores Y, the matrix of y-vectors;
        // c          sy, of dimension m x m, stores S'Y;
        // c          ss, of dimension m x m, stores S'S;
        // c          yy, of dimension m x m, stores Y'Y;
        // c          wt, of dimension m x m, stores the Cholesky factorization
        // c                                  of (theta*S'S+LD^(-1)L'); see eq.
        // c                                  (2.26) in [3].
        // c
        // c     wn is a double precision working array of dimension 2m x 2m
        // c       used to store the LEL^T factorization of the indefinite matrix
        // c                 K = [-D -Y'ZZ'Y/theta     L_a'-R_z'  ]
        // c                     [L_a -R_z           theta*S'AA'S ]
        // c
        // c       where     E = [-I  0]
        // c                     [ 0  I]
        // c
        // c     snd is a double precision working array of dimension 2m x 2m
        // c       used to store the lower triangular part of
        // c                 N = [Y' ZZ'Y   L_a'+R_z']
        // c                     [L_a +R_z  S'AA'S   ]
        // c            
        // c     z(n),r(n),d(n),t(n), xp(n),wa(8*m) are double precision working ar
        // c       z  is used at different times to store the Cauchy point and
        // c          the Newton point.
        // c       xp is used to safeguard the projected Newton direction
        // c
        // c     sg(m),sgo(m),yg(m),ygo(m) are double precision working arrays. 
        // c
        // c     index is an integer working array of dimension n.
        // c       In subroutine freev, index is used to store the free and fixed
        // c          variables at the Generalized Cauchy Point (GCP).
        // c
        // c     iwhere is an integer working array of dimension n used to record
        // c       the status of the vector x for GCP computation.
        // c       iwhere(i)=0 or -3 if x(i) is free and has bounds,
        // c                 1       if x(i) is fixed at l(i), and l(i) .ne. u(i)
        // c                 2       if x(i) is fixed at u(i), and u(i) .ne. l(i)
        // c                 3       if x(i) is always fixed, i.e.,  u(i)=x(i)=l(i)
        // c                -1       if x(i) is always free, i.e., no bounds on it.
        // c
        // c     indx2 is an integer working array of dimension n.
        // c       Within subroutine cauchy, indx2 corresponds to the array iorder.
        // c       In subroutine freev, a list of variables entering and leaving
        // c       the free set is stored in indx2, and it is passed on to
        // c       subroutine formk with this information.
        // c
        // c     task is a working string of characters of length 60 indicating
        // c       the current job when entering and leaving this subroutine.
        // c
        // c     iprint is an INTEGER variable that must be set by the user.
        // c       It controls the frequency and type of output generated:
        // c        iprint<0    no output is generated;
        // c        iprint=0    print only one line at the last iteration;
        // c        0<iprint<99 print also f and |proj g| every iprint iterations;

        // c        iprint=99   print details of every iteration except n-vectors;

        // c        iprint=100  print also the changes of active set and final x;
        // c        iprint>100  print details of every iteration including x and g;
        // c       When iprint > 0, the file iterate.dat will be created to
        // c                        summarize the iteration.
        // c
        // c     csave is a working string of characters of length 60.
        // c
        // c     lsave is a logical working array of dimension 4.
        // c
        // c     isave is an integer working array of dimension 23.
        // c
        // c     dsave is a double precision working array of dimension 29.
        // c
        // c
        // c     Subprograms called
        // c
        // c       L-BFGS-B Library ... cauchy, subsm, lnsrlb, formk, 
        // c
        // c        errclb, prn1lb, prn2lb, prn3lb, active, projgr,
        // c
        // c        freev, cmprlb, matupd, formt.
        // c
        // c       Minpack2 Library ... timer
        // c
        // c       Linpack Library ... dcopy, 
        // c
        // c
        // c     References:
        // c
        // c       [1] R. H. Byrd, P. Lu, J. Nocedal and C. Zhu, ``A limited
        // c       memory algorithm for bound constrained optimization'',
        // c       SIAM J. Scientific Computing 16 (1995), no. 5, pp. 1190--1208.
        // c
        // c       [2] C. Zhu, R.H. Byrd, P. Lu, J. Nocedal, ``L-BFGS-B: FORTRAN
        // c       Subroutines for Large Scale Bound Constrained Optimization''
        // c       Tech. Report, NAM-11, EECS Department, Northwestern University,

        // c       1994.
        // c 
        // c       [3] R. Byrd, J. Nocedal and R. Schnabel "Representations of
        // c       Quasi-Newton Matrices and their use in Limited Memory Methods'',
        // c       Mathematical Programming 63 (1994), no. 4, pp. 129-156.
        // c
        // c       (Postscript files of these papers are available via anonymous
        // c        ftp to eecs.nwu.edu in the directory pub/lbfgs/lbfgs_bcm.)
        // c
        // c                           *  *  *
        // c
        // c     NEOS, November 1994. (Latest revision June 1996.)
        // c     Optimization Technology Center.
        // c     Argonne National Laboratory and Northwestern University.
        // c     Written by
        // c                        Ciyou Zhu
        // c     in collaboration with R.H. Byrd, P. Lu-Chen and J. Nocedal.
        // c
        // c
        // c     ************
        // 
        // 
        private static void Mainlb(int n,
            int m,
            double[] x, int xOffset,
            double[] l, int lOffset,
            double[] u, int uOffset,
            int[] nbd, int nbdOffset,
            ref double f,
            double[] g, int gOffset,
            double factr,
            double pgtol,
            double[] ws, int wsOffset,
            double[] wy, int wyOffset,
            double[] sy, int syOffset,
            double[] ss, int ssOffset,
            double[] wt, int wtOffset,
            double[] wn, int wnOffset,
            double[] snd, int sndOffset,
            double[] z, int zOffset,
            double[] r, int rOffset,
            double[] d, int dOffset,
            double[] t, int tOffset,
            double[] xp, int xpOffset,
            double[] wa, int waOffset,
            int[] index, int indexOffset,
            int[] iwhere, int iwhereOffset,
            int[] indx2, int indx2Offset,
            ref Task task,
            int iprint,
            ref Task csave,
            bool[] lsave, int lsaveOffset,
            int[] isave, int isaveOffset,
            double[] dsave, int dsaveOffset)
        {

            bool prjctd = false;
            bool cnstnd = false;
            bool boxed = false;
            bool updatd = false;
            bool wrk = false;
            int i = 0;
            int k = 0;
            int nintol = 0;
            int itfile = 0;
            int iback = 0;
            int nskip = 0;
            int head = 0;
            int col = 0;
            int iter = 0;
            int itail = 0;
            int iupdat = 0;
            int nseg = 0;
            int nfgv = 0;
            int info = 0;
            int ifun = 0;
            int iword = 0;
            int nfree = 0;
            int nact = 0;
            int ileave = 0;
            int nenter = 0;
            double theta = 0.0d;
            double fold = 0.0d;
            double dr = 0.0d;
            double rr = 0.0d;
            double tol = 0.0d;
            double xstep = 0.0d;
            double sbgnrm = 0.0d;
            double ddum = 0.0d;
            double dnorm = 0.0d;
            double dtd = 0.0d;
            double epsmch = 0.0d;
            double cpu1 = 0.0d;
            double cpu2 = 0.0d;
            double cachyt = 0.0d;
            double sbtime = 0.0d;
            double lnscht = 0.0d;
            double time1 = 0.0d;
            // double time2 = 0.0d;
            double gd = 0.0d;
            double gdold = 0.0d;
            double stp = 0.0d;
            double stpmx = 0.0d;
            // double time = 0.0d;
            // float epsilon = 0.0f;

            if (task == Task.Start)
            {
                // 
                // epsmch = (double)(Epsilon.epsilon(1.0));
                epsmch = 1.11022302462516E-16;

                // 
                //Timer.timer(time1);
                // 
                // c        Initialize counters and scalars when task='START'.
                // 
                // c           for the limited memory BFGS matrices:
                col = 0;
                head = 1;
                theta = 1.0;
                iupdat = 0;
                updatd = false;
                iback = 0;
                itail = 0;
                iword = 0;
                nact = 0;
                ileave = 0;
                nenter = 0;
                fold = 0.0;
                dnorm = 0.0;
                cpu1 = 0.0;
                gd = 0.0;
                stpmx = 0.0;
                sbgnrm = 0.0;
                stp = 0.0;
                gdold = 0.0;
                dtd = 0.0;
                // 
                // c           for operation counts:
                iter = 0;
                nfgv = 0;
                nseg = 0;
                nintol = 0;
                nskip = 0;
                nfree = n;
                ifun = 0;
                // c           for stopping tolerance:
                tol = factr * epsmch;
                // 
                // c           for measuring running time:
                cachyt = 0;
                sbtime = 0;
                lnscht = 0;

                // 
                // c           'info' records the termination information.
                info = 0;
                // 
                itfile = 8;
                if (iprint >= 1)
                {
                    // c                                open a summary file 'iterate.dat'
                    ; // WARNING: Unimplemented statement in Fortran source.
                }
                // 
                // c        Check the input arguments for errors.
                // 
                Errclb(n, m, factr, l, lOffset, u, uOffset,
                    nbd, nbdOffset, ref task, ref info, ref k);

                if (task == Task.Error)
                {
                    /*
                    Prn3lb.prn3lb(n, x, _x_offset, f, task, iprint,
                        info, itfile, iter, nfgv, nintol, nskip, nact, sbgnrm, 0.0, nseg, word,
                        iback, stp, xstep, k, cachyt, sbtime, lnscht);
                    */
                    return;
                }

                // 
                // Prn1lb.prn1lb(n, m, l, _l_offset, u, _u_offset, x, _x_offset, iprint, itfile, epsmch);

                // 
                // c        Initialize iwhere & project x onto the feasible set.
                // 
                Active(n, l, lOffset, u, uOffset, nbd, nbdOffset, x,
                    xOffset, iwhere, iwhereOffset, out prjctd, out cnstnd, out boxed);
                // 
                // c        The end of the initialization.
                // 
            }
            else
            {
                // c          restore local variables.
                // 
                prjctd = lsave[1 - 1 + lsaveOffset];
                cnstnd = lsave[2 - 1 + lsaveOffset];
                boxed = lsave[3 - 1 + lsaveOffset];
                updatd = lsave[4 - 1 + lsaveOffset];
                // 
                nintol = isave[1 - 1 + isaveOffset];
                itfile = isave[3 - 1 + isaveOffset];
                iback = isave[4 - 1 + isaveOffset];
                nskip = isave[5 - 1 + isaveOffset];
                head = isave[6 - 1 + isaveOffset];
                col = isave[7 - 1 + isaveOffset];
                itail = isave[8 - 1 + isaveOffset];
                iter = isave[9 - 1 + isaveOffset];
                iupdat = isave[10 - 1 + isaveOffset];
                nseg = isave[12 - 1 + isaveOffset];
                nfgv = isave[13 - 1 + isaveOffset];
                info = isave[14 - 1 + isaveOffset];
                ifun = isave[15 - 1 + isaveOffset];
                iword = isave[16 - 1 + isaveOffset];
                nfree = isave[17 - 1 + isaveOffset];
                nact = isave[18 - 1 + isaveOffset];
                ileave = isave[19 - 1 + isaveOffset];
                nenter = isave[20 - 1 + isaveOffset];
                // 
                theta = dsave[1 - 1 + dsaveOffset];
                fold = dsave[2 - 1 + dsaveOffset];
                tol = dsave[3 - 1 + dsaveOffset];
                dnorm = dsave[4 - 1 + dsaveOffset];
                epsmch = dsave[5 - 1 + dsaveOffset];
                cpu1 = dsave[6 - 1 + dsaveOffset];
                cachyt = dsave[7 - 1 + dsaveOffset];
                sbtime = dsave[8 - 1 + dsaveOffset];
                lnscht = dsave[9 - 1 + dsaveOffset];
                time1 = dsave[10 - 1 + dsaveOffset];
                gd = dsave[11 - 1 + dsaveOffset];
                stpmx = dsave[12 - 1 + dsaveOffset];
                sbgnrm = dsave[13 - 1 + dsaveOffset];
                stp = dsave[14 - 1 + dsaveOffset];
                gdold = dsave[15 - 1 + dsaveOffset];
                dtd = dsave[16 - 1 + dsaveOffset];
                // 
                // c        After returning from the driver go to the point where execution
                // c        is to resume.
                // 
                if (task == Task.FgLN)
                {
                    goto L666;
                }

                if (task == Task.NewX)
                {
                    goto L777;
                }

                if (task == Task.FgSt)
                {
                    goto L111;
                }
            }

            // 
            // c     Compute f0 and g0.
            // 
            task = Task.FgSt;

            // c          return to the driver to calculate f and g; reenter at 111.
            goto L1000;

        L111:

            nfgv = 1;

            // 
            // c     Compute the infinity norm of the (-) projected gradient.
            // 
            Projgr(n, l, lOffset, u, uOffset, nbd, nbdOffset,
                x, xOffset, g, gOffset, ref sbgnrm);

            if (sbgnrm <= pgtol)
            {
                // terminate the algorithm.
                task = Task.Convergence;
                goto L999;
            }
        // 
        // c ----------------- the beginning of the loop --------------------------
        // 
        L222:
            iword = -1;
            //Compute the Generalized Cauchy Point (GCP).
            Cauchy(n, x, xOffset, l, lOffset, u, uOffset, nbd, nbdOffset,
                g, gOffset, indx2, indx2Offset, iwhere, iwhereOffset, t, tOffset,
                d, dOffset, z, zOffset, m, wy, wyOffset, ws, wsOffset, sy, syOffset,
                wt, wtOffset, theta, col, head, wa, 1 - 1 + waOffset, wa,
                2 * m + 1 - 1 + waOffset, wa, 4 * m + 1 - 1 + waOffset, wa,
                6 * m + 1 - 1 + waOffset, ref nseg, sbgnrm, ref info, epsmch);

            cachyt = cachyt + cpu2 - cpu1;
            nintol += nseg;
            // Count the entering and leaving variables for iter > 0; 
            // find the index set of free and active variables at the GCP.
            Freev(n, ref nfree, index, indexOffset, ref nenter, ref ileave, indx2, indx2Offset,
                iwhere, iwhereOffset, ref wrk, updatd, cnstnd, iprint, iter);
            nact = n - nfree;
            // If there are no free variables or B=theta*I, 
            // then skip the subspace minimization.
            // 
            if (nfree == 0 || col == 0) {
                goto L555;
            }
            // c     Subspace minimization.
            // c     Form  the LEL^T factorization of the indefinite
            // c       matrix    K = [-D -Y'ZZ'Y/theta     L_a'-R_z'  ]
            // c                     [L_a -R_z           theta*S'AA'S ]
            // c       where     E = [-I  0]
            // c                     [ 0  I]
            if (wrk) {
                Formk(n, nfree, index, indexOffset, nenter,
                    ileave, indx2, indx2Offset, iupdat, updatd, wn, wnOffset,
                    snd, sndOffset, m, ws, wsOffset, wy, wyOffset, sy, syOffset,
                    theta, col, head, ref info);
            }
            // compute r=-Z'B(xcp-xk)-Z'g   (using wa(2m+1)=W'(xcp-x) from 'cauchy')
            Cmprlb(n, m, x, xOffset, g, gOffset, ws, wsOffset, wy, wyOffset,
                sy, syOffset, wt, wtOffset, z, zOffset, r, rOffset, wa, waOffset,
                index, indexOffset, theta, col, head, nfree, cnstnd, ref info);
            // c-jlm-jn   call the direct method. 
            Subsm(n, m, nfree, index, indexOffset, l, lOffset, u, uOffset,
                nbd, nbdOffset, z, zOffset, r, rOffset, xp, xpOffset, ws, wsOffset,
                wy, wyOffset, theta, x, xOffset, g, gOffset, col, head,
                ref iword, wa, waOffset, wn, wnOffset, iprint, ref info);          

        L555:
            // c     Line search and optimality tests.
            // c     Generate the search direction d:=z-x.
            for (i = 1; i <= n; i++) {
                d[i - 1 + dOffset] = z[i - 1 + zOffset] - x[i - 1 + xOffset];
            }

        L666:
            Lnsrlb(n, l, lOffset, u, uOffset, nbd, nbdOffset, x, xOffset,
                f, ref fold, ref gd, ref gdold, g, gOffset, d, dOffset, r, rOffset, t, tOffset,
                z, zOffset, ref stp, ref dnorm, ref dtd, ref xstep, ref stpmx, iter, ref ifun,
                ref iback, ref nfgv, ref info, ref task, boxed, cnstnd, ref csave, isave,
                22 - 1 + isaveOffset, dsave, 17 - 1 + dsaveOffset);

            if (iback >= 20)
            {
                // restore the previous iterate.
                Dcopy(n, t, tOffset, 1, x, xOffset, 1);
                Dcopy(n, r, rOffset, 1, g, gOffset, 1);
                f = fold;
                if (col == 0)
                {
                    info = -9;
                    // restore the actual number of f and g evaluations etc.
                    nfgv -= 1;
                    ifun -= 1;
                    iback -= 1;
                    task = Task.Abnormal;
                    iter += 1;
                    goto L999;
                }

                nfgv -= 1;
                info = 0;
                col = 0;
                head = 1;
                theta = 1.0;
                iupdat = 0;
                updatd = false;
                task = Task.RestartLN;
                goto L222;
            }

            if (task == Task.FgLN)
            {
                // return to the driver for calculating f and g; reenter at 666.
                goto L1000;
            }
            // calculate and print out the quantities related to the new X.
            iter += 1;
            // Compute the infinity norm of the projected (-)gradient.
            Projgr(n, l, lOffset, u, uOffset, nbd, nbdOffset, x, xOffset, g, gOffset, ref sbgnrm);
            goto L1000;

            L777:
            // c     Test for termination.
            if (sbgnrm <= pgtol)
            {
                // terminate the algorithm.
                task = Task.Convergence;
                goto L999;
            }

            ddum = System.Math.Max(System.Math.Abs(fold), System.Math.Max(System.Math.Abs(f), 1.0));

            if (fold - f <= tol * ddum)
            {
                // terminate the algorithm.
                task = Task.Convergence;

                if (iback >= 10)
                {
                    info = -5;
                }

                // i.e., to issue a warning if iback>10 in the line search.
                goto L999;
            }

            // 
            // Compute d=newx-oldx, r=newg-oldg, rr=y'y and dr=y's.
            // 
            for (i = 1; i <= n; i++)
            {
                r[i - 1 + rOffset] = g[i - 1 + gOffset] - r[i - 1 + rOffset];
            }

            rr = Ddot(n, r, rOffset, 1, r, rOffset, 1);

            if (stp == 1.0)
            {
                dr = gd - gdold;
                ddum = -gdold;
            }
            else
            {
                dr = (gd - gdold) * stp;
                Dscal(n, stp, d, dOffset, 1);
                ddum = -(gdold * stp);
            }

            if (dr <= epsmch * ddum)
            {
                // skip the L-BFGS update.
                nskip += 1;
                updatd = false;


                if (iprint >= 1)
                {
                    // DISPLAY: dr, ddum
                    // DISPLAY: '  ys=',1p,e10.3,'  -gs=',1P,E10.3,' BFGS update SKIPPED'"
                }

                goto L888;
            }

            // 
            // cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
            // c
            // c     Update the L-BFGS matrix.
            // c
            // cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
            // 
            updatd = true;
            iupdat += 1;

            // 
            // c     Update matrices WS and WY and form the middle matrix in B.
            // 
            Matupd(n, m, ws, wsOffset, wy, wyOffset, sy, syOffset,
                ss, ssOffset, d, dOffset, r, rOffset, ref itail, iupdat, ref col,
                ref head, ref theta, rr, dr, stp, dtd);

            // 
            // c     Form the upper half of the pds T = theta*SS + L*D^(-1)*L';
            // c        Store T in the upper triangular of the array wt;
            // c        Cholesky factorize T to J*J' with
            // c           J' stored in the upper triangular of wt.
            // 
            Formt(m, wt, wtOffset, sy, syOffset, ss,
                ssOffset, col, theta, ref info);

        // 
        //   Now the inverse of the middle matrix in B is
        // 
        //   [  D^(1/2)      O ] [ -D^(1/2)  D^(-1/2)*L' ]
        //   [ -L*D^(-1/2)   J ] [  0        J'          ]
        // 
        L888:
            // 
            // c -------------------- the end of the loop -----------------------------
            // 
            goto L222;

        L999:
        

        L1000:
            // 
            //   Save local variables.
            // 
            lsave[1 - 1 + lsaveOffset] = prjctd;
            lsave[2 - 1 + lsaveOffset] = cnstnd;
            lsave[3 - 1 + lsaveOffset] = boxed;
            lsave[4 - 1 + lsaveOffset] = updatd;
            // 
            isave[1 - 1 + isaveOffset] = nintol;
            isave[3 - 1 + isaveOffset] = itfile;
            isave[4 - 1 + isaveOffset] = iback;
            isave[5 - 1 + isaveOffset] = nskip;
            isave[6 - 1 + isaveOffset] = head;
            isave[7 - 1 + isaveOffset] = col;
            isave[8 - 1 + isaveOffset] = itail;
            isave[9 - 1 + isaveOffset] = iter;
            isave[10 - 1 + isaveOffset] = iupdat;
            isave[12 - 1 + isaveOffset] = nseg;
            isave[13 - 1 + isaveOffset] = nfgv;
            isave[14 - 1 + isaveOffset] = info;
            isave[15 - 1 + isaveOffset] = ifun;
            isave[16 - 1 + isaveOffset] = iword;
            isave[17 - 1 + isaveOffset] = nfree;
            isave[18 - 1 + isaveOffset] = nact;
            isave[19 - 1 + isaveOffset] = ileave;
            isave[20 - 1 + isaveOffset] = nenter;
            // 
            dsave[1 - 1 + dsaveOffset] = theta;
            dsave[2 - 1 + dsaveOffset] = fold;
            dsave[3 - 1 + dsaveOffset] = tol;
            dsave[4 - 1 + dsaveOffset] = dnorm;
            dsave[5 - 1 + dsaveOffset] = epsmch;
            dsave[6 - 1 + dsaveOffset] = cpu1;
            dsave[7 - 1 + dsaveOffset] = cachyt;
            dsave[8 - 1 + dsaveOffset] = sbtime;
            dsave[9 - 1 + dsaveOffset] = lnscht;
            dsave[10 - 1 + dsaveOffset] = time1;
            dsave[11 - 1 + dsaveOffset] = gd;
            dsave[12 - 1 + dsaveOffset] = stpmx;
            dsave[13 - 1 + dsaveOffset] = sbgnrm;
            dsave[14 - 1 + dsaveOffset] = stp;
            dsave[15 - 1 + dsaveOffset] = gdold;
            dsave[16 - 1 + dsaveOffset] = dtd;

            return;
        }

        // 
        // c======================= The end of lnsrlb =============================
        // 
        // 
        // 
        // c     ************
        // c
        // c     Subroutine matupd
        // c
        // c       This subroutine updates matrices WS and WY, and forms the
        // c         middle matrix in B.
        // c
        // c     Subprograms called:
        // c
        // c       Linpack ... dcopy, 
        // c
        // c
        // c                           *  *  *
        // c
        // c     NEOS, November 1994. (Latest revision June 1996.)
        // c     Optimization Technology Center.
        // c     Argonne National Laboratory and Northwestern University.
        // c     Written by
        // c                        Ciyou Zhu
        // c     in collaboration with R.H. Byrd, P. Lu-Chen and J. Nocedal.
        // c
        // c
        // c     ************
        // 
        // 
        // c     Set pointers for matrices WS and WY.
        // 
        internal static void Matupd(int n,
            int m, double[] ws, int wsOffset, double[] wy, int wyOffset,
            double[] sy, int syOffset, double[] ss, int ssOffset, double[] d, int dOffset,
            double[] r, int rOffset, ref int itail, int iupdat, ref int col,
            ref int head, ref double theta, double rr, double dr, double stp, double dtd)
        {

            int j = 0;
            int pointr = 0;

            if (iupdat <= m)
            {
                col = iupdat;
                itail = (head + iupdat - 2) % m + 1;
            }
            else
            {
                itail = itail % m + 1;
                head = head % m + 1;
            }

            // 
            // c     Update matrices WS and WY.
            // 
            Dcopy(n, d, dOffset, 1, ws, 1 - 1 + (itail - 1) * n + wsOffset, 1);
            Dcopy(n, r, rOffset, 1, wy, 1 - 1 + (itail - 1) * n + wyOffset, 1);

            // 
            // c     Set theta=yy/ys.
            // 
            theta = rr / dr;
            // 
            // c     Form the middle matrix in B.
            // 
            // c        update the upper triangle of SS,
            // c                                         and the lower triangle of SY:

            if (iupdat > m)
            {
                // c                              move old information
                {
                    for (j = 1; j <= col - 1; j++)
                    {
                        Dcopy(j, ss, 2 - 1 + (j + 1 - 1) * m
                                           + ssOffset, 1, ss, 1 - 1 + (j - 1) * m + ssOffset, 1);
                        Dcopy(col - j, sy, j + 1 - 1 + (j + 1
                                                        - 1) * m + syOffset, 1, sy, j - 1 + (j - 1) * m + syOffset, 1);
                    }
                }
            }
            // c        add new information: the last row of SY
            // c                                             and the last column of SS:
            pointr = head;
            {
                for (j = 1; j <= col - 1; j++)
                {
                    sy[col - 1 + (j - 1) * m + syOffset] =
                        Ddot(n,
                          d, dOffset, 1,
                        wy, 1 - 1 + (pointr - 1) * n + wyOffset, 1);

                    ss[j - 1 + (col - 1) * m + ssOffset] =
                        Ddot(n,
                          ws, 1 - 1 + (pointr - 1) * n + wsOffset, 1,
                          d, dOffset, 1);

                    pointr = pointr % m + 1;
                }
            }
            if (stp == 1.0)
            {
                ss[col - 1 + (col - 1) * m + ssOffset] = dtd;
            }
            else
            {
                ss[col - 1 + (col - 1) * m + ssOffset] = stp * stp * dtd;
            }

            sy[col - 1 + (col - 1) * m + syOffset] = dr;

            return;
        }

        // 
        // c======================= The end of prn3lb =============================
        // 
        // 
        // 
        // c     ************
        // c
        // c     Subroutine projgr
        // c
        // c     This subroutine computes the infinity norm of the projected
        // c       gradient.
        // c
        // c
        // c                           *  *  *
        // c
        // c     NEOS, November 1994. (Latest revision June 1996.)
        // c     Optimization Technology Center.
        // c     Argonne National Laboratory and Northwestern University.
        // c     Written by
        // c                        Ciyou Zhu
        // c     in collaboration with R.H. Byrd, P. Lu-Chen and J. Nocedal.
        // c
        // c
        // c     ************
        // 
        // 
        internal static void Projgr(int n,
        double[] l, int lOffset,
        double[] u, int uOffset,
        int[] nbd, int nbdOffset,
        double[] x, int xOffset,
        double[] g, int gOffset,
        ref double sbgnrm)
        {

            int i = 0;
            double gi = 0.0d;
            sbgnrm = 0.0;
            {
                for (i = 1; i <= n; i++)
                {
                    gi = g[i - 1 + gOffset];
                    if (nbd[i - 1 + nbdOffset] != 0)
                    {
                        if (gi < 0.0)
                        {
                            if (nbd[i - 1 + nbdOffset] >= 2)
                            {
                                gi = System.Math.Max(x[i - 1 + xOffset] - u[i - 1 + uOffset], gi);
                            }
                        }
                        else
                        {
                            if (nbd[i - 1 + nbdOffset] <= 2)
                            {
                                gi = System.Math.Min(x[i - 1 + xOffset] - l[i - 1 + lOffset], gi);
                            }
                        }
                    }
                    sbgnrm = System.Math.Max(sbgnrm, System.Math.Abs(gi));
                }
            }
        }

        // c                                                                       
        // c  L-BFGS-B is released under the �New BSD License� (aka �Modified
        // c  or �3-clause license�)                                           
        // c  Please read attached file License.txt                                
        // c                                        
        // c===========   L-BFGS-B (version 3.0.  April 25, 2011  =================
        // c
        // c     This is a modified version of L-BFGS-B. Minor changes in the updat
        // c     code appear preceded by a line comment as follows 
        // c  
        // c     c-jlm-jn 
        // c
        // c     Major changes are described in the accompanying paper:
        // c
        // c         Jorge Nocedal and Jose Luis Morales, Remark on "Algorithm 778:
        // c         L-BFGS-B: Fortran Subroutines for Large-Scale Bound Constraine
        // c         Optimization"  (2011). To appear in  ACM Transactions on 
        // c         Mathematical Software,
        // c
        // c     The paper describes an improvement and a correction to Algorithm 7
        // c     It is shown that the performance of the algorithm can be improved 
        // c     significantly by making a relatively simple modication to the subs
        // c     minimization phase. The correction concerns an error caused by the
        // c     of routine dpmeps to estimate machine precision. 
        // c
        // c     The total work space **wa** required by the new version is 
        // c 
        // c                  2*m*n + 11m*m + 5*n + 8*m 
        // c
        // c     the old version required 
        // c
        // c                  2*m*n + 12m*m + 4*n + 12*m 
        // c
        // c
        // c            J. Nocedal  Department of Electrical Engineering and
        // c                        Computer Science.
        // c                        Northwestern University. Evanston, IL. USA
        // c
        // c
        // c           J.L Morales  Departamento de Matematicas, 
        // c                        Instituto Tecnologico Autonomo de Mexico
        // c                        Mexico D.F. Mexico.
        // c
        // c                        March  2011    
        // c                                                 
        // c=======================================================================
        // 
        // c
        // c-jlm-jn
        // 
        // 
        // c     ************
        // c
        // c     Subroutine setulb
        // c
        // c     This subroutine partitions the working arrays wa and iwa, and 
        // c       then uses the limited memory BFGS method to solve the bound
        // c       constrained optimization problem by calling mainlb.
        // c       (The direct method will be used in the subspace minimization.)
        // c
        // c     n is an integer variable.
        // c       On entry n is the dimension of the problem.
        // c       On exit n is unchanged.
        // c
        // c     m is an integer variable.
        // c       On entry m is the maximum number of variable metric corrections
        // c
        // c         used to define the limited memory matrix.
        // c       On exit m is unchanged.
        // c
        // c     x is a double precision array of dimension n.
        // c       On entry x is an approximation to the solution.
        // c       On exit x is the current approximation.
        // c
        // c     l is a double precision array of dimension n.
        // c       On entry l is the lower bound on x.
        // c       On exit l is unchanged.
        // c
        // c     u is a double precision array of dimension n.
        // c       On entry u is the upper bound on x.
        // c       On exit u is unchanged.
        // c
        // c     nbd is an integer array of dimension n.
        // c       On entry nbd represents the type of bounds imposed on the
        // c         variables, and must be specified as follows:
        // c         nbd(i)=0 if x(i) is unbounded,
        // c                1 if x(i) has only a lower bound,
        // c                2 if x(i) has both lower and upper bounds, and
        // c                3 if x(i) has only an upper bound.
        // c       On exit nbd is unchanged.
        // c
        // c     f is a double precision variable.
        // c       On first entry f is unspecified.
        // c       On final exit f is the value of the function at x.
        // c
        // c     g is a double precision array of dimension n.
        // c       On first entry g is unspecified.
        // c       On final exit g is the value of the gradient at x.
        // c
        // c     factr is a double precision variable.
        // c       On entry factr >= 0 is specified by the user.  The iteration
        // c         will stop when
        // c
        // c         (f^k - f^{k+1})/max{|f^k|,|f^{k+1}|,1} <= factr*epsmch
        // c
        // c         where epsmch is the machine precision, which is automatically
        // c         generated by the code. Typical values for factr: 1.d+12 for
        // c         low accuracy; 1.d+7 for moderate accuracy; 1.d+1 for extremely
        // c         high accuracy.
        // c       On exit factr is unchanged.
        // c
        // c     pgtol is a double precision variable.
        // c       On entry pgtol >= 0 is specified by the user.  The iteration
        // c         will stop when
        // c
        // c                 max{|proj g_i | i = 1, ..., n} <= pgtol
        // c
        // c         where pg_i is the ith component of the projected gradient.   

        // c       On exit pgtol is unchanged.
        // c
        // c     wa is a double precision working array of length 
        // c       (2mmax + 5)nmax + 12mmax^2 + 12mmax.
        // c
        // c     iwa is an integer working array of length 3nmax.
        // c
        // c     task is a working string of characters of length 60 indicating
        // c       the current job when entering and quitting this subroutine.
        // c
        // c     iprint is an integer variable that must be set by the user.
        // c       It controls the frequency and type of output generated:
        // c        iprint<0    no output is generated;
        // c        iprint=0    print only one line at the last iteration;
        // c        0<iprint<99 print also f and |proj g| every iprint iterations;
        // c        iprint=99   print details of every iteration except n-vectors;
        // c        iprint=100  print also the changes of active set and final x;
        // c        iprint>100  print details of every iteration including x and g;
        // c       When iprint > 0, the file iterate.dat will be created to
        // c                        summarize the iteration.
        // c
        // c     csave is a working string of characters of length 60.
        // c
        // c     lsave is a logical working array of dimension 4.
        // c       On exit with 'task' = NEW_X, the following information is 
        // c                                                             available:
        // c         If lsave(1) = .true.  then  the initial X has been replaced by
        // c                                     its projection in the feasible set
        // c         If lsave(2) = .true.  then  the problem is constrained;
        // c         If lsave(3) = .true.  then  each variable has upper and lower
        // c                                     bounds;
        // c
        // c     isave is an integer working array of dimension 44.
        // c       On exit with 'task' = NEW_X, the following information is 
        // c                                                             available:
        // c         isave(22) = the total number of intervals explored in the 
        // c                         search of Cauchy points;
        // c         isave(26) = the total number of skipped BFGS updates before 
        // c                         the current iteration;
        // c         isave(30) = the number of current iteration;
        // c         isave(31) = the total number of BFGS updates prior the current
        // c                         iteration;
        // c         isave(33) = the number of intervals explored in the search of
        // c                         Cauchy point in the current iteration;
        // c         isave(34) = the total number of function and gradient 
        // c                         evaluations;
        // c         isave(36) = the number of function value or gradient
        // c                                  evaluations in the current iteration;
        // c         if isave(37) = 0  then the subspace argmin is within the box;
        // c         if isave(37) = 1  then the subspace argmin is beyond the box;
        // c         isave(38) = the number of free variables in the current
        // c                         iteration;
        // c         isave(39) = the number of active constraints in the current
        // c                         iteration;
        // c         n + 1 - isave(40) = the number of variables leaving the set of
        // c                           active constraints in the current iteration;
        // c         isave(41) = the number of variables entering the set of active
        // c                         constraints in the current iteration.
        // c
        // c     dsave is a double precision working array of dimension 29.
        // c       On exit with 'task' = NEW_X, the following information is
        // c                                                             available:
        // c         dsave(1) = current 'theta' in the BFGS matrix;
        // c         dsave(2) = f(x) in the previous iteration;
        // c         dsave(3) = factr*epsmch;
        // c         dsave(4) = 2-norm of the line search direction vector;
        // c         dsave(5) = the machine precision epsmch generated by the code;
        // c         dsave(7) = the accumulated time spent on searching for
        // c                                                         Cauchy points;
        // c         dsave(8) = the accumulated time spent on
        // c                                                 subspace minimization;
        // c         dsave(9) = the accumulated time spent on line search;
        // c         dsave(11) = the slope of the line search function at
        // c                                  the current point of line search;
        // c         dsave(12) = the maximum relative step length imposed in
        // c                                                           line search;
        // c         dsave(13) = the infinity norm of the projected gradient;
        // c         dsave(14) = the relative step length in the line search;
        // c         dsave(15) = the slope of the line search function at
        // c                                 the starting point of the line search;
        // c         dsave(16) = the square of the 2-norm of the line search
        // c                                                      direction vector.
        // c
        // c     Subprograms called:
        // c
        // c       L-BFGS-B Library ... mainlb.    
        // c
        // c
        // c     References:
        // c
        // c       [1] R. H. Byrd, P. Lu, J. Nocedal and C. Zhu, ``A limited
        // c       memory algorithm for bound constrained optimization'',
        // c       SIAM J. Scientific Computing 16 (1995), no. 5, pp. 1190--1208.
        // c
        // c       [2] C. Zhu, R.H. Byrd, P. Lu, J. Nocedal, ``L-BFGS-B: a
        // c       limited memory FORTRAN code for solving bound constrained
        // c       optimization problems'', Tech. Report, NAM-11, EECS Department,
        // c       Northwestern University, 1994.
        // c
        // c       (Postscript files of these papers are available via anonymous
        // c        ftp to eecs.nwu.edu in the directory pub/lbfgs/lbfgs_bcm.)
        // c
        // c                           *  *  *
        // c
        // c     NEOS, November 1994. (Latest revision June 1996.)
        // c     Optimization Technology Center.
        // c     Argonne National Laboratory and Northwestern University.
        // c     Written by
        // c                        Ciyou Zhu
        // c     in collaboration with R.H. Byrd, P. Lu-Chen and J. Nocedal.
        // c
        // c
        // c     ************
        // 
        internal static void Setulb(int n,
        int m,
        double[] x, int xOffset,
        double[] l, int lOffset,
        double[] u, int uOffset,
        int[] nbd, int nbdOffset,
        ref double f,
        double[] g, int gOffset,
        double factr,
        double pgtol,
        double[] wa, int waOffset,
        int[] iwa, int iwaOffset,
        ref Task task,
        int iprint,
        ref Task csave,
        bool[] lsave, int lsaveOffset,
        int[] isave, int isaveOffset,
        double[] dsave, int dsaveOffset)
        {

            int lws = 0;
            int lr = 0;
            int lz = 0;
            int lt = 0;
            int ld = 0;
            int lxp = 0;
            int lwa = 0;
            int lwy = 0;
            int lsy = 0;
            int lss = 0;
            int lwt = 0;
            int lwn = 0;
            int lsnd = 0;

            if (task == Task.Start)
            {
                isave[1 - 1 + isaveOffset] = m * n;
                isave[2 - 1 + isaveOffset] = (int)System.Math.Pow(m, 2);
                isave[3 - 1 + isaveOffset] = 4 * (int)System.Math.Pow(m, 2);
                isave[4 - 1 + isaveOffset] = 1;
                isave[5 - 1 + isaveOffset] = isave[4 - 1 + isaveOffset] + isave[1 - 1 + isaveOffset];
                isave[6 - 1 + isaveOffset] = isave[5 - 1 + isaveOffset] + isave[1 - 1 + isaveOffset];
                isave[7 - 1 + isaveOffset] = isave[6 - 1 + isaveOffset] + isave[2 - 1 + isaveOffset];
                isave[8 - 1 + isaveOffset] = isave[7 - 1 + isaveOffset] + isave[2 - 1 + isaveOffset];
                isave[9 - 1 + isaveOffset] = isave[8 - 1 + isaveOffset] + isave[2 - 1 + isaveOffset];
                isave[10 - 1 + isaveOffset] = isave[9 - 1 + isaveOffset] + isave[3 - 1 + isaveOffset];
                isave[11 - 1 + isaveOffset] = isave[10 - 1 + isaveOffset] + isave[3 - 1 + isaveOffset];
                isave[12 - 1 + isaveOffset] = isave[11 - 1 + isaveOffset] + n;
                isave[13 - 1 + isaveOffset] = isave[12 - 1 + isaveOffset] + n;
                isave[14 - 1 + isaveOffset] = isave[13 - 1 + isaveOffset] + n;
                isave[15 - 1 + isaveOffset] = isave[14 - 1 + isaveOffset] + n;
                isave[16 - 1 + isaveOffset] = isave[15 - 1 + isaveOffset] + n;
            }
            lws = isave[4 - 1 + isaveOffset];
            lwy = isave[5 - 1 + isaveOffset];
            lsy = isave[6 - 1 + isaveOffset];
            lss = isave[7 - 1 + isaveOffset];
            lwt = isave[8 - 1 + isaveOffset];
            lwn = isave[9 - 1 + isaveOffset];
            lsnd = isave[10 - 1 + isaveOffset];
            lz = isave[11 - 1 + isaveOffset];
            lr = isave[12 - 1 + isaveOffset];
            ld = isave[13 - 1 + isaveOffset];
            lt = isave[14 - 1 + isaveOffset];
            lxp = isave[15 - 1 + isaveOffset];
            lwa = isave[16 - 1 + isaveOffset];
            // 
            Mainlb(n, m, x, xOffset, l, lOffset, u, uOffset, nbd,
                nbdOffset, ref f, g, gOffset, factr, pgtol, wa, lws - 1 + waOffset,
                wa, lwy - 1 + waOffset, wa, lsy - 1 + waOffset, wa,
                lss - 1 + waOffset, wa, lwt - 1 + waOffset, wa,
                lwn - 1 + waOffset, wa, lsnd - 1 + waOffset, wa,
                lz - 1 + waOffset, wa, lr - 1 + waOffset, wa,
                ld - 1 + waOffset, wa, lt - 1 + waOffset, wa,
                lxp - 1 + waOffset, wa, lwa - 1 + waOffset, iwa,
                1 - 1 + iwaOffset, iwa, n + 1 - 1 + iwaOffset,
                iwa, 2 * n + 1 - 1 + iwaOffset, ref task, iprint, ref csave,
                lsave, lsaveOffset, isave, 22 - 1 + isaveOffset, dsave,
                dsaveOffset);
        }

        // 
        // c======================= The end of projgr =============================
        // 
        // 
        // c     ******************************************************************
        // c
        // c     This routine contains the major changes in the updated version.
        // c     The changes are described in the accompanying paper
        // c
        // c      Jose Luis Morales, Jorge Nocedal
        // c      "Remark On Algorithm 788: L-BFGS-B: Fortran Subroutines for Large
        // c       Bound Constrained Optimization". Decemmber 27, 2010.
        // c
        // c             J.L. Morales  Departamento de Matematicas, 
        // c                           Instituto Tecnologico Autonomo de Mexico
        // c                           Mexico D.F.
        // c
        // c             J, Nocedal    Department of Electrical Engineering and
        // c                           Computer Science.
        // c                           Northwestern University. Evanston, IL. USA
        // c
        // c                           January 17, 2011
        // c
        // c      *****************************************************************
        // c                           
        // c
        // c     Subroutine subsm
        // c
        // c     Given xcp, l, u, r, an index set that specifies
        // c       the active set at xcp, and an l-BFGS matrix B 
        // c       (in terms of WY, WS, SY, WT, head, col, and theta), 
        // c       this subroutine computes an approximate solution
        // c       of the subspace problem
        // c
        // c       (P)   min Q(x) = r'(x-xcp) + 1/2 (x-xcp)' B (x-xcp)
        // c
        // c             subject to l<=x<=u
        // c                       x_i=xcp_i for all i in A(xcp)
        // c                     
        // c       along the subspace unconstrained Newton direction 
        // c       
        // c          d = -(Z'BZ)^(-1) r.
        // c
        // c       The formula for the Newton direction, given the L-BFGS matrix
        // c       and the Sherman-Morrison formula, is
        // c
        // c          d = (1/theta)r + (1/theta*2) Z'WK^(-1)W'Z r.
        // c 
        // c       where
        // c                 K = [-D -Y'ZZ'Y/theta     L_a'-R_z'  ]
        // c                     [L_a -R_z           theta*S'AA'S ]
        // c
        // c     Note that this procedure for computing d differs 
        // c     from that described in [1]. One can show that the matrix K is
        // c     equal to the matrix M^[-1]N in that paper.
        // c
        // c     n is an integer variable.
        // c       On entry n is the dimension of the problem.
        // c       On exit n is unchanged.
        // c
        // c     m is an integer variable.
        // c       On entry m is the maximum number of variable metric corrections

        // c         used to define the limited memory matrix.
        // c       On exit m is unchanged.
        // c
        // c     nsub is an integer variable.
        // c       On entry nsub is the number of free variables.
        // c       On exit nsub is unchanged.
        // c
        // c     ind is an integer array of dimension nsub.
        // c       On entry ind specifies the coordinate indices of free variables.
        // c       On exit ind is unchanged.
        // c
        // c     l is a double precision array of dimension n.
        // c       On entry l is the lower bound of x.
        // c       On exit l is unchanged.
        // c
        // c     u is a double precision array of dimension n.
        // c       On entry u is the upper bound of x.
        // c       On exit u is unchanged.
        // c
        // c     nbd is a integer array of dimension n.
        // c       On entry nbd represents the type of bounds imposed on the
        // c         variables, and must be specified as follows:
        // c         nbd(i)=0 if x(i) is unbounded,
        // c                1 if x(i) has only a lower bound,
        // c                2 if x(i) has both lower and upper bounds, and
        // c                3 if x(i) has only an upper bound.
        // c       On exit nbd is unchanged.
        // c
        // c     x is a double precision array of dimension n.
        // c       On entry x specifies the Cauchy point xcp. 
        // c       On exit x(i) is the minimizer of Q over the subspace of
        // c                                                        free variables.
        // c
        // c     d is a double precision array of dimension n.
        // c       On entry d is the reduced gradient of Q at xcp.
        // c       On exit d is the Newton direction of Q. 
        // c
        // c    xp is a double precision array of dimension n.
        // c       used to safeguard the projected Newton direction 
        // c
        // c    xx is a double precision array of dimension n
        // c       On entry it holds the current iterate
        // c       On output it is unchanged
        // 
        // c    gg is a double precision array of dimension n
        // c       On entry it holds the gradient at the current iterate
        // c       On output it is unchanged
        // c
        // c     ws and wy are double precision arrays;
        // c     theta is a double precision variable;
        // c     col is an integer variable;
        // c     head is an integer variable.
        // c       On entry they store the information defining the
        // c                                          limited memory BFGS matrix:
        // c         ws(n,m) stores S, a set of s-vectors;
        // c         wy(n,m) stores Y, a set of y-vectors;
        // c         theta is the scaling factor specifying B_0 = theta I;
        // c         col is the number of variable metric corrections stored;
        // c         head is the location of the 1st s- (or y-) vector in S (or Y).
        // c       On exit they are unchanged.
        // c
        // c     iword is an integer variable.
        // c       On entry iword is unspecified.
        // c       On exit iword specifies the status of the subspace solution.
        // c         iword = 0 if the solution is in the box,
        // c                 1 if some bound is encountered.
        // c
        // c     wv is a double precision working array of dimension 2m.
        // c
        // c     wn is a double precision array of dimension 2m x 2m.
        // c       On entry the upper triangle of wn stores the LEL^T factorization
        // c         of the indefinite matrix
        // c
        // c              K = [-D -Y'ZZ'Y/theta     L_a'-R_z'  ]
        // c                  [L_a -R_z           theta*S'AA'S ]
        // c                                                    where E = [-I  0]
        // c                                                              [ 0  I]
        // c       On exit wn is unchanged.
        // c
        // c     iprint is an INTEGER variable that must be set by the user.
        // c       It controls the frequency and type of output generated:
        // c        iprint<0    no output is generated;
        // c        iprint=0    print only one line at the last iteration;
        // c        0<iprint<99 print also f and |proj g| every iprint iterations;

        // c        iprint=99   print details of every iteration except n-vectors;

        // c        iprint=100  print also the changes of active set and final x;
        // c        iprint>100  print details of every iteration including x and g;
        // c       When iprint > 0, the file iterate.dat will be created to
        // c                        summarize the iteration.
        // c
        // c     info is an integer variable.
        // c       On entry info is unspecified.
        // c       On exit info = 0       for normal return,
        // c                    = nonzero for abnormal return 
        // c                                  when the matrix K is ill-conditioned.
        // c
        // c     Subprograms called:
        // c
        // c       Linpack 
        // c
        // c
        // c     References:
        // c
        // c       [1] R. H. Byrd, P. Lu, J. Nocedal and C. Zhu, ``A limited
        // c       memory algorithm for bound constrained optimization'',
        // c       SIAM J. Scientific Computing 16 (1995), no. 5, pp. 1190--1208.
        // c
        // c
        // c
        // c                           *  *  *
        // c
        // c     NEOS, November 1994. (Latest revision June 1996.)
        // c     Optimization Technology Center.
        // c     Argonne National Laboratory and Northwestern University.
        // c     Written by
        // c                        Ciyou Zhu
        // c     in collaboration with R.H. Byrd, P. Lu-Chen and J. Nocedal.
        // c
        // c
        // c     ************
        // 
        // c
        // 
        internal static void Subsm(int n,
        int m, int nsub, int[] ind, int indOffset,
        double[] l, int lOffset, double[] u, int uOffset,
        int[] nbd, int nbdOffset, double[] x, int xOffset,
        double[] d, int dOffset, double[] xp, int xpOffset,
        double[] ws, int wsOffset, double[] wy, int wyOffset,
        double theta, double[] xx, int xxOffset, double[] gg, int ggOffset,
        int col, int head, ref int iword,
        double[] wv, int wvOffset, double[] wn, int wnOffset, int iprint, ref int info)
        {
            int pointr = 0;
            int m2 = 0;
            int col2 = 0;
            int ibd = 0;
            int jy = 0;
            int js = 0;
            int i = 0;
            int j = 0;
            int k = 0;
            double alpha = 0.0d;
            double xk = 0.0d;
            double dk = 0.0d;
            double temp1 = 0.0d;
            double temp2 = 0.0d;
            double ddP = 0.0d;

            if (nsub <= 0)
            {
                return;
            }

            // 
            // Compute wv = W'Zd.
            // 
            pointr = head;
            {
                for (i = 1; i <= col; i++)
                {
                    temp1 = 0.0;
                    temp2 = 0.0;
                    {
                        for (j = 1; j <= nsub; j++)
                        {
                            k = ind[j - 1 + indOffset];
                            temp1 += wy[k - 1 + (pointr - 1)
                                * n + wyOffset] * d[j - 1 + dOffset];
                            temp2 += ws[k - 1 + (pointr - 1)
                                * n + wsOffset] * d[j - 1 + dOffset];
                        }
                    }

                    wv[i - 1 + wvOffset] = temp1;
                    wv[col + i - 1 + wvOffset] = theta * temp2;
                    pointr = pointr % m + 1;
                }
            }

            // 
            // Compute wv:=K^(-1)wv.
            // 
            m2 = 2 * m;
            col2 = 2 * col;

            Dtrsl(wn, wnOffset, m2, col2, wv, wvOffset, 11, out info);

            {
                for (i = 1; i <= col; i++)
                {
                    wv[i - 1 + wvOffset] = -wv[i - 1 + wvOffset];
                }
            }

            Dtrsl(wn, wnOffset, m2, col2, wv, wvOffset, 1, out info);

            // 
            // Compute d = (1/theta)d + (1/theta**2)Z'W wv.
            // 
            pointr = head;
            {
                for (jy = 1; jy <= col; jy++)
                {
                    js = col + jy;
                    {
                        for (i = 1; i <= nsub; i++)
                        {
                            k = ind[i - 1 + indOffset];
                            d[i - 1 + dOffset] = d[i - 1 + dOffset]
                                                   + wy[k - 1 + (pointr - 1) * n + wyOffset] * wv[jy - 1
                                                       + wvOffset] / theta + ws[k - 1
                                                       + (pointr - 1) * n + wsOffset] * wv[js - 1 + wvOffset];
                        }
                    }

                    pointr = pointr % m + 1;
                }
            }


            Dscal(nsub, 1.0 / theta, d, dOffset, 1);

            //  
            // ----------------------------------------------------
            // Let us try the projection, d is the Newton direction
            // 
            iword = 0;

            Dcopy(n, x, xOffset, 1, xp, xpOffset, 1);

            // c
            {
                for (i = 1; i <= nsub; i++)
                {
                    k = ind[i - 1 + indOffset];
                    dk = d[i - 1 + dOffset];
                    xk = x[k - 1 + xOffset];
                    if (nbd[k - 1 + nbdOffset] != 0)
                    {
                        if (nbd[k - 1 + nbdOffset] == 1)
                        {
                            x[k - 1 + xOffset] = System.Math.Max(l[k - 1 + lOffset], xk + dk);
                            if (x[k - 1 + xOffset] == l[k - 1 + lOffset])
                            {
                                iword = 1;
                            }
                        }
                        else
                        {
                            if (nbd[k - 1 + nbdOffset] == 2)
                            {
                                xk = System.Math.Max(l[k - 1 + lOffset], xk + dk);
                                x[k - 1 + xOffset] = System.Math.Min(u[k - 1 + uOffset], xk);
                                if (x[k - 1 + xOffset] == l[k - 1
                                                              + lOffset] || x[k - 1 + xOffset] == u[k - 1 + uOffset])
                                {
                                    iword = 1;
                                }
                            }
                            else
                            {
                                if (nbd[k - 1 + nbdOffset] == 3)
                                {
                                    x[k - 1 + xOffset] = System.Math.Min(u[k - 1 + uOffset], xk + dk);
                                    if (x[k - 1 + xOffset] == u[k - 1 + uOffset])
                                    {
                                        iword = 1;
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        x[k - 1 + xOffset] = xk + dk;
                    }
                }
            }


            if (iword == 0)
            {
                goto L911;
            }

            // 
            // check sign of the directional derivative
            // 
            ddP = 0.0;
            {
                for (i = 1; i <= n; i++)
                {
                    ddP += (x[i - 1 + xOffset]
                            - xx[i - 1 + xxOffset]) * gg[i - 1 + ggOffset];
                }
            }

            if (ddP > 0.0)
            {
                Dcopy(n, xp, xpOffset, 1, x, xOffset, 1);

                // DISPLAY: " Positive dir derivative in projection "
                // DISPLAY: " Using the backtracking step "
            }
            else
            {
                goto L911;
            }

            // 
            // -----------------------------------------------------------------
            // 
            alpha = 1.0;
            temp1 = alpha;
            ibd = 0;
            {
                for (i = 1; i <= nsub; i++)
                {
                    k = ind[i - 1 + indOffset];
                    dk = d[i - 1 + dOffset];
                    if (nbd[k - 1 + nbdOffset] != 0)
                    {
                        if (dk < 0.0 && nbd[k - 1 + nbdOffset] <= 2)
                        {
                            temp2 = l[k - 1 + lOffset] - x[k - 1 + xOffset];
                            if (temp2 >= 0.0)
                            {
                                temp1 = 0.0;
                            }
                            else if (dk * alpha < temp2)
                            {
                                temp1 = temp2 / dk;
                            }
                        }
                        else if (dk > 0.0 && nbd[k - 1 + nbdOffset] >= 2)
                        {
                            temp2 = u[k - 1 + uOffset] - x[k - 1 + xOffset];
                            if (temp2 <= 0.0)
                            {
                                temp1 = 0.0;
                            }
                            else if (dk * alpha > temp2)
                            {
                                temp1 = temp2 / dk;
                            }
                        }

                        if (temp1 < alpha)
                        {
                            alpha = temp1;
                            ibd = i;
                        }
                    }
                }
            }

            if (alpha < 1.0)
            {
                dk = d[ibd - 1 + dOffset];
                k = ind[ibd - 1 + indOffset];
                if (dk > 0.0)
                {
                    x[k - 1 + xOffset] = u[k - 1 + uOffset];
                    d[ibd - 1 + dOffset] = 0.0;
                }
                else if (dk < 0.0)
                {
                    x[k - 1 + xOffset] = l[k - 1 + lOffset];
                    d[ibd - 1 + dOffset] = 0.0;
                }
            }
            {
                for (i = 1; i <= nsub; i++)
                {
                    k = ind[i - 1 + indOffset];
                    x[k - 1 + xOffset] += alpha * d[i - 1 + dOffset];
                }
            }


        L911:

            if (iprint >= 99)
            {
                // DISPLAY: "----------------exit SUBSM --------------------"
            }

            return;
        }

        // c                                                                       
        // c  L-BFGS-B is released under the �New BSD License� (aka �Modified
        // c  or �3-clause license�)                                           
        // c  Please read attached file License.txt                                
        // c                                        
        // c
        // c     dpofa factors a double precision symmetric positive definite
        // c     matrix.
        // c
        // c     dpofa is usually called by dpoco, but it can be called
        // c     directly with a saving in time if  rcond  is not needed.
        // c     (time for dpoco) = (1 + 18/n)*(time for dpofa) .
        // c
        // c     on entry
        // c
        // c        a       double precision(lda, n)
        // c                the symmetric matrix to be factored.  only the
        // c                diagonal and upper triangle are used.
        // c
        // c        lda     integer
        // c                the leading dimension of the array  a .
        // c
        // c        n       integer
        // c                the order of the matrix  a .
        // c
        // c     on return
        // c
        // c        a       an upper triangular matrix  r  so that  a = trans(r)*r

        // c                where  trans(r)  is the transpose.
        // c                the strict lower triangle is unaltered.
        // c                if  info .ne. 0 , the factorization is not complete.
        // c
        // c        info    integer
        // c                = 0  for normal return.
        // c                = k  signals an error condition.  the leading minor
        // c                     of order  k  is not positive definite.
        // c
        // c     linpack.  this version dated 08/14/78 .
        // c     cleve moler, university of new mexico, argonne national lab.
        // c
        // c     subroutines and functions
        // c
        // c     blas ddot
        // c     fortran sqrt
        // c
        // c     internal variables
        // c
        // c     begin block with ...exits to 40
        // c
        // c
        private static void Dpofa(double[] a, int aOffset, int lda, int n, ref int info)
        {
            double t = 0.0d;
            double s = 0.0d;
            int j = 0;
            int jm1 = 0;
            int k = 0;
            {
                for (j = 1; j <= n; j++)
                {
                    info = j;
                    s = 0.0e0;
                    jm1 = j - 1;
                    if (jm1 < 1)
                    {
                        goto L20;
                    }

                    {
                        for (k = 1; k <= jm1; k++)
                        {
                            t = a[k - 1 + (j - 1) * lda + aOffset]
                                - Ddot(k - 1, a, 1 - 1 + (k - 1) * lda
                                                       + aOffset, 1, a, 1 - 1 + (j - 1) * lda + aOffset, 1);

                            t /= a[k - 1 + (k - 1) * lda + aOffset];
                            a[k - 1 + (j - 1) * lda + aOffset] = t;
                            s += t * t;
                        }
                    }

                L20:
                    s = a[j - 1 + (j - 1) * lda + aOffset] - s;
                    // c     ......exit
                    if (s <= 0.0e0)
                    {
                        goto L40;
                    }

                    a[j - 1 + (j - 1) * lda + aOffset] = System.Math.Sqrt(s);
                }
            }

            info = 0;

        L40:
            return;
        }

        // *     .. Scalar Arguments ..
        // *     ..
        // *     .. Array Arguments ..
        // *     ..
        // *
        // *  Purpose
        // *  =======
        // **
        // *     scales a vector by a constant.
        // *     uses unrolled loops for increment equal to one.
        // *     jack dongarra, linpack, 3/11/78.
        // *     modified 3/93 to return if incx .le. 0.
        // *     modified 12/3/93, array(1) declarations changed to array(*)
        // *
        // *
        // *     .. Local Scalars ..
        // *     ..
        // *     .. Intrinsic Functions ..
        // *     ..
        private static void Dscal(int n, double da, double[] dx, int dxOffset, int incx)
        {

            int i = 0;
            int m = 0;
            int mp1 = 0;
            int nincx = 0;

            if (n <= 0 || incx <= 0)
            {
                return;
            }

            if (incx == 1)
            {
                goto L20;
            }

            // *
            // *        code for increment not equal to 1
            // *
            nincx = n * incx;
            {
                int iInc = incx;
                for (i = 1; iInc < 0 ? i >= nincx : i <= nincx; i += iInc)
                {
                    dx[i - 1 + dxOffset] = da * dx[i - 1 + dxOffset];
                }
            }
            return;
        // *
        // *        code for increment equal to 1
        // *
        // *
        // *        clean-up loop
        // *
        L20:
            m = n % 5;

            if (m == 0)
            {
                goto L40;
            }
            {
                for (i = 1; i <= m; i++)
                {
                    dx[i - 1 + dxOffset] = da * dx[i - 1 + dxOffset];
                }
            }

            if (n < 5)
            {
                return;
            }

        L40:
            mp1 = m + 1;
            {
                int iInc = 5;
                for (i = mp1; i <= n; i += iInc)
                {
                    dx[i - 1 + dxOffset] = da * dx[i - 1 + dxOffset];
                    dx[i + 1 - 1 + dxOffset] = da * dx[i + 1 - 1 + dxOffset];
                    dx[i + 2 - 1 + dxOffset] = da * dx[i + 2 - 1 + dxOffset];
                    dx[i + 3 - 1 + dxOffset] = da * dx[i + 3 - 1 + dxOffset];
                    dx[i + 4 - 1 + dxOffset] = da * dx[i + 4 - 1 + dxOffset];
                }
            }

            return;
        }

        // *     .. Scalar Arguments ..
        // *     ..
        // *     .. Array Arguments ..
        // *     ..
        // *
        // *  Purpose
        // *  =======
        // *
        // *     forms the dot product of two vectors.
        // *     uses unrolled loops for increments equal to one.
        // *     jack dongarra, linpack, 3/11/78.
        // *     modified 12/3/93, array(1) declarations changed to array(*)
        // *
        // *
        // *     .. Local Scalars ..
        // *     ..
        // *     .. Intrinsic Functions ..
        // *     ..
        private static double Ddot(int n, double[] dx, int dxOffset,
           int incx, double[] dy, int dyOffset, int incy)
        {
            double dtemp = 0.0d;
            int i = 0;
            int ix = 0;
            int iy = 0;
            int m = 0;
            int mp1 = 0;
            double ddot = 0.0d;
            ddot = 0.0e0;
            dtemp = 0.0e0;
            if (n <= 0)
            {
                return ddot;
            }

            if (incx == 1 && incy == 1)
            {
                goto L20;
            }
            // *
            // *        code for unequal increments or equal increments
            // *          not equal to 1
            // *
            ix = 1;
            iy = 1;
            if (incx < 0)
            {
                ix = (-n + 1) * incx + 1;
            }
            if (incy < 0)
            {
                iy = (-n + 1) * incy + 1;
            }

            {
                for (i = 1; i <= n; i++)
                {
                    dtemp += dx[ix - 1 + dxOffset] * dy[iy - 1 + dyOffset];
                    ix += incx;
                    iy += incy;
                }
            }
            ddot = dtemp;
            return ddot;

        // *
        // *        code for both increments equal to 1
        // *
        // *
        // *        clean-up loop
        // *
        L20:
            m = n % 5;

            if (m == 0)
            {
                goto L40;
            }

            {
                for (i = 1; i <= m; i++)
                {
                    dtemp += dx[i - 1 + dxOffset] * dy[i - 1 + dyOffset];
                }
            }

            if (n < 5)
            {
                goto L60;
            }

        L40:
            mp1 = m + 1;
            {
                int iInc = 5;
                for (i = mp1; i <= n; i += iInc)
                {
                    dtemp = dtemp + dx[i - 1 + dxOffset] * dy[i - 1
                                                                + dyOffset] + dx[i + 1 - 1 + dxOffset] * dy[i + 1 - 1
                        + dyOffset] + dx[i + 2 - 1 + dxOffset] * dy[i + 2 - 1
                                                                        + dyOffset] + dx[i + 3 - 1 + dxOffset] * dy[i + 3 - 1
                        + dyOffset] + dx[i + 4 - 1 + dxOffset] * dy[i + 4 - 1 + dyOffset];
                }
            }
        L60:
            ddot = dtemp;
            return ddot;
        }

        private static void Dcopy(int n, double[] dx, int dxOffset, int incx,
           double[] dy, int dyOffset, int incy)
        {

            int i = 0;
            int ix = 0;
            int iy = 0;
            int m = 0;
            int mp1 = 0;
            if (n <= 0)
            {
                return;
            }

            if (incx == 1 && incy == 1)
            {
                goto L20;
            }
            // c
            // c        code for unequal increments or equal increments
            // c          not equal to 1
            // c
            ix = 1;
            iy = 1;
            if (incx < 0)
            {
                ix = (-n + 1) * incx + 1;
            }
            if (incy < 0)
            {
                iy = (-n + 1) * incy + 1;
            }
            {
                for (i = 1; i <= n; i++)
                {
                    dy[iy - 1 + dyOffset] = dx[ix - 1 + dxOffset];
                    ix += incx;
                    iy += incy;
                }
            }
            return;
        // c
        // c        code for both increments equal to 1
        // c
        // c
        // c        clean-up loop
        // c
        L20:
            m = n % 7;
            if (m == 0)
            {
                goto L40;
            }
            {
                for (i = 1; i <= m; i++)
                {
                    dy[i - 1 + dyOffset] = dx[i - 1 + dxOffset];
                }
            }
            if (n < 7)
            {
                return;
            }

        L40:
            mp1 = m + 1;
            {
                int iInc = 7;
                for (i = mp1; i <= n; i += iInc)
                {
                    dy[i - 1 + dyOffset] = dx[i - 1 + dxOffset];
                    dy[i + 1 - 1 + dyOffset] = dx[i + 1 - 1 + dxOffset];
                    dy[i + 2 - 1 + dyOffset] = dx[i + 2 - 1 + dxOffset];
                    dy[i + 3 - 1 + dyOffset] = dx[i + 3 - 1 + dxOffset];
                    dy[i + 4 - 1 + dyOffset] = dx[i + 4 - 1 + dxOffset];
                    dy[i + 5 - 1 + dyOffset] = dx[i + 5 - 1 + dxOffset];
                    dy[i + 6 - 1 + dyOffset] = dx[i + 6 - 1 + dxOffset];
                }
            }
        }

        // *     .. Scalar Arguments ..
        // *     ..
        // *     .. Array Arguments ..
        // *     ..
        // *
        // *  Purpose
        // *  =======
        // *
        // *     DAXPY constant times a vector plus a vector.
        // *     uses unrolled loops for increments equal to one.
        // *
        // *  Further Details
        // *  ===============
        // *
        // *     jack dongarra, linpack, 3/11/78.
        // *     modified 12/3/93, array(1) declarations changed to array(*)
        // *
        // *  =====================================================================
        // *
        // *     .. Local Scalars ..
        // *     ..
        // *     .. Intrinsic Functions ..
        // *     ..
        private static void Daxpy(int n, double da,
             double[] dx, int dxOffset, int incx, double[] dy, int dyOffset, int incy)
        {

            int i = 0;
            int ix = 0;
            int iy = 0;
            int m = 0;
            int mp1 = 0;
            if (n <= 0)
            {
                return;
            }

            if (da == 0.0e0)
            {
                return;
            }

            if (incx == 1 && incy == 1)
            {
                // *
                // *        code for both increments equal to 1
                // *
                // *
                // *        clean-up loop
                // *
                m = n % 4;
                if (m != 0)
                {
                    {
                        for (i = 1; i <= m; i++)
                        {
                            dy[i - 1 + dyOffset] += da * dx[i - 1 + dxOffset];
                        }
                    }
                }

                if (n < 4)
                {
                    return;
                }

                mp1 = m + 1;
                {
                    int iInc = 4;
                    for (i = mp1; i <= n; i += iInc)
                    {
                        dy[i - 1 + dyOffset] += da * dx[i - 1 + dxOffset];
                        dy[i + 1 - 1 + dyOffset] += da * dx[i + 1 - 1 + dxOffset];
                        dy[i + 2 - 1 + dyOffset] += da * dx[i + 2 - 1 + dxOffset];
                        dy[i + 3 - 1 + dyOffset] += da * dx[i + 3 - 1 + dxOffset];
                    }
                }
            }
            else
            {
                // *
                // *        code for unequal increments or equal increments
                // *          not equal to 1
                // *
                ix = 1;
                iy = 1;
                if (incx < 0)
                {
                    ix = (-n + 1) * incx + 1;
                }
                if (incy < 0)
                {
                    iy = (-n + 1) * incy + 1;
                }
                {
                    for (i = 1; i <= n; i++)
                    {
                        dy[iy - 1 + dyOffset] += da * dx[ix - 1 + dxOffset];
                        ix += incx;
                        iy += incy;
                    }
                }
            }
            return;
        }
    }

}