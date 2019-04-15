#ifndef CORRELATIONGRID_H
#define CORRELATIONGRID_H

/*
 * 这个类是用来生成临时的似然场，然后来做CSM的。
*/

#include <stdint.h>


typedef unsigned int  uint32_t;
typedef int            int32_t;
typedef unsigned char  uint8_t;
typedef char            int8_t;



/**
   * Implementation of a correlation grid used for scan matching
   */
  class CorrelationGrid
  {

  public:
    /**
     * Destructor
     */
    virtual ~CorrelationGrid()
    {
      delete [] m_pKernel;
      delete [] m_pGrid;
    }

  public:
    /**
     * Create a correlation grid of given size and parameters
     * @param width
     * @param height
     * @param resolution
     * @param smearDeviation
     * @return correlation grid
     */
    static CorrelationGrid* CreateGrid(int width,
                                       int height,
                                       double resolution,
                                       double smearDeviation)
    {
      assert(resolution != 0.0);

      // +1 in case of roundoff
      unsigned int borderSize = GetHalfKernelSize(smearDeviation, resolution) + 1;

      CorrelationGrid* pGrid = new CorrelationGrid(width, height, borderSize, resolution, smearDeviation);

      return pGrid;
    }

    /**
     * Gets the index into the data pointer of the given grid coordinate
     * @param rGrid
     * @param boundaryCheck
     * @return grid index
     */
    virtual int GridIndex(const Vector2<int32_t>& rGrid, kt_bool boundaryCheck = true) const
    {
      int32_t x = rGrid.GetX() + m_Roi.GetX();
      int32_t y = rGrid.GetY() + m_Roi.GetY();

      return Grid<uint8_t>::GridIndex(Vector2<int32_t>(x, y), boundaryCheck);
    }

    /**
     * Get the Region Of Interest (ROI)
     * @return region of interest
     */
    inline const Rectangle2<int32_t>& GetROI() const
    {
      return m_Roi;
    }

    /**
     * Sets the Region Of Interest (ROI)
     * @param roi
     */
    inline void SetROI(const Rectangle2<int32_t>& roi)
    {
      m_Roi = roi;
    }

    /**
     * Smear cell if the cell at the given point is marked as "occupied"
     * 对一个点进行模糊 可以认为是似然场的膨胀
     * 似然场是一个m_KernelSize*m_KernelSize的矩形区域
     * 这个kernel实际上是事先计算好了，就存储在m_pKernel里面
     * 因此这里面实际上直接就从m_pKernel里面查询就可以了。
     * @param rGridPoint
     */
    inline void SmearPoint(const Vector2<int32_t>& rGridPoint)
    {
      assert(m_pKernel != NULL);

      int gridIndex = GridIndex(rGridPoint);
      if (GetDataPointer()[gridIndex] != GridStates_Occupied)
      {
        return;
      }

      int32_t halfKernel = m_KernelSize / 2;

      // apply kernel
      for (int32_t j = -halfKernel; j <= halfKernel; j++)
      {
        uint8_t* pGridAdr = GetDataPointer(Vector2<int32_t>(rGridPoint.GetX(), rGridPoint.GetY() + j));

        int32_t kernelConstant = (halfKernel) + m_KernelSize * (j + halfKernel);

        // if a point is on the edge of the grid, there is no problem
        // with running over the edge of allowable memory, because
        // the grid has margins to compensate for the kernel size
        for (int32_t i = -halfKernel; i <= halfKernel; i++)
        {
          int32_t kernelArrayIndex = i + kernelConstant;

          uint8_t kernelValue = m_pKernel[kernelArrayIndex];
          if (kernelValue > pGridAdr[i])
          {
            // kernel value is greater, so set it to kernel value
            pGridAdr[i] = kernelValue;
          }
        }
      }
    }

  protected:
    /**
     * Constructs a correlation grid of given size and parameters
     * @param width
     * @param height
     * @param borderSize
     * @param resolution
     * @param smearDeviation
}
     */
    CorrelationGrid(uint32_t width, uint32_t height, uint32_t borderSize,
                    double resolution, double smearDeviation)
      : m_SmearDeviation(smearDeviation),m_resolution(resolution)
      , m_pKernel(NULL)
    {
      m_pGrid = new uint8_t[width * height];

      // setup region of interest
      m_Roi = Rectangle2<int32_t>(borderSize, borderSize, width, height);

      // calculate kernel
      CalculateKernel();
    }

    /**
     * Sets up the kernel for grid smearing.
     * 计算用来生成似然场的kernel
     */
    virtual void CalculateKernel()
    {
      double resolution = GetResolution();

      assert(resolution != 0.0);
      assert(m_SmearDeviation != 0.0);

      // min and max distance deviation for smearing;
      // will smear for two standard deviations, so deviation must be at least 1/2 of the resolution
      const double MIN_SMEAR_DISTANCE_DEVIATION = 0.5 * resolution;
      const double MAX_SMEAR_DISTANCE_DEVIATION = 10 * resolution;

      // check if given value too small or too big
      if (!math::InRange(m_SmearDeviation, MIN_SMEAR_DISTANCE_DEVIATION, MAX_SMEAR_DISTANCE_DEVIATION))
      {
        std::stringstream error;
        error << "Mapper Error:  Smear deviation too small:  Must be between "
              << MIN_SMEAR_DISTANCE_DEVIATION
              << " and "
              << MAX_SMEAR_DISTANCE_DEVIATION;
        throw std::runtime_error(error.str());
      }

      // NOTE:  Currently assumes a two-dimensional kernel

      // +1 for center
      m_KernelSize = 2 * GetHalfKernelSize(m_SmearDeviation, resolution) + 1;

      // allocate kernel
      m_pKernel = new uint8_t[m_KernelSize * m_KernelSize];
      if (m_pKernel == NULL)
      {
        throw std::runtime_error("Unable to allocate memory for kernel!");
      }

      // calculate kernel
      int32_t halfKernel = m_KernelSize / 2;
      for (int32_t i = -halfKernel; i <= halfKernel; i++)
      {
        for (int32_t j = -halfKernel; j <= halfKernel; j++)
        {
#ifdef WIN32
          double distanceFromMean = _hypot(i * resolution, j * resolution);
#else
          double distanceFromMean = hypot(i * resolution, j * resolution);
#endif
          double z = exp(-0.5 * pow(distanceFromMean / m_SmearDeviation, 2));

          uint32_t kernelValue = static_cast<uint32_t>(math::Round(z * GridStates_Occupied));
          assert(math::IsUpTo(kernelValue, static_cast<uint32_t>(255)));

          int kernelArrayIndex = (i + halfKernel) + m_KernelSize * (j + halfKernel);
          m_pKernel[kernelArrayIndex] = static_cast<uint8_t>(kernelValue);
        }
      }
    }

    /**
     * Computes the kernel half-size based on the smear distance and the grid resolution.
     * Computes to two standard deviations to get 95% region and to reduce aliasing.
     * @param smearDeviation
     * @param resolution
     * @return kernel half-size based on the parameters
     */
    static int GetHalfKernelSize(double smearDeviation, double resolution)
    {
      assert(resolution != 0.0);

      return static_cast<int32_t>(math::Round(2.0 * smearDeviation / resolution));
    }

  private:
    /**
     * The point readings are smeared by this value in X and Y to create a smoother response.
     * Default value is 0.03 meters.
     * 用来模糊 或者说用来计算似然场的标准差
     */
    double m_SmearDeviation;

    //这个地图的分辨率
    double m_resolution;

    // Size of one side of the kernel
    int32_t m_KernelSize;

    //用来存储地图信息的Grid
    uint8_t* m_pGrid;

    // Cached kernel for smearing
    // 用来进行模糊的kernel
    uint8_t* m_pKernel;

    // region of interest
    Rectangle2<int32_t> m_Roi;
  };  // CorrelationGrid




#endif // CORRELATIONGRID_H
