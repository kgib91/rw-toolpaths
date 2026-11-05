using System.Text.Json;
using Clipper2Lib;
using RW.Toolpaths.Milling;
using Xunit;
namespace RW.Toolpaths.Tests;
public sealed class NativeContinuousClearanceTests
{
 [Fact] public void ContinuousGuardMatchesIndependentHoleWallAndDepthControls()
 {
  using var document=JsonDocument.Parse(Fixture);
  var profile=new VCarveCutterProfile(new[]{new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere,0,1.5875,1.5875),new CarvingCutterSection(CarvingCutterSectionKind.Linear,1.5875,3,20)},12.7);
  foreach(var item in document.RootElement.GetProperty("cases").EnumerateArray())
  {
   var c=item.GetProperty("controls").EnumerateArray().Select(q=>new Point3D(q[0].GetDouble(),q[1].GetDouble(),q[2].GetDouble())).ToArray();
   var move=ToolMove.CubicBezier(c[0],c[1],c[2],c[3],.3);
   var boundary=item.GetProperty("boundary").EnumerateArray().Select(r=>(IReadOnlyList<PointD>)r.EnumerateArray().Select(q=>new PointD(q[0].GetDouble(),q[1].GetDouble())).ToArray()).ToArray();
   bool actual=VCarveNativeCutterProof.Certified(move,new VCarveNativeContainment(boundary,profile),profile,out var reason);
   Assert.True(actual==item.GetProperty("expectedSafe").GetBoolean(),item.GetProperty("name").GetString()+": "+reason);
  }
 }
 [Fact] public void DetailFilteringRequiresExplicitOperationAllowance()
 {
  Point3D[] source=[new(0,0,5),new(.3,0,4.96),new(.6,0,5)];
  Assert.Equal(source,RasterMicrotextureFilter.Apply(source,1.2,new CarvingOptions().MicrotextureLiftAllowanceMm).Points);
  Assert.Equal(2,RasterMicrotextureFilter.Apply(source,1.2,.05).Points.Length);
 }
 [Fact] public void CompletionUsesCurrentOperationAllowanceInsteadOfGlobalFinishRule()
 {
  var target=new[]{(IReadOnlyList<PointD>)new PointD[]{new(0,0),new(.1,0),new(.1,.02),new(0,.02)}};
  var profile=new VCarveCutterProfile(new[]{new CarvingCutterSection(CarvingCutterSectionKind.Linear,0,1,1)},1);
  var fine=new VCarveOptions{PathToleranceMm=.01};var coarse=fine with{PathToleranceMm=.08};
  Assert.False(VCarveCoverageCertificate.Certify(target,Array.Empty<TaggedToolpath>(),profile,gapRadius:fine.CompletionGapRadiusMm).Certified);
  Assert.True(VCarveCoverageCertificate.Certify(target,Array.Empty<TaggedToolpath>(),profile,gapRadius:coarse.CompletionGapRadiusMm).Certified);
 }
 private const string Fixture = """
{
  "cases": [
    {
      "name": "safe-curved-interior",
      "controls": [
        [
          0,
          0.2,
          -0.0008
        ],
        [
          0.3333333333333333,
          0.8,
          -0.0008
        ],
        [
          0.6666666666666666,
          0.8,
          -0.0008
        ],
        [
          1,
          0.2,
          -0.0008
        ]
      ],
      "boundary": [
        [
          [
            -2,
            -1
          ],
          [
            2,
            -1
          ],
          [
            2,
            1
          ],
          [
            -2,
            1
          ]
        ]
      ],
      "expectedSafe": true,
      "witnessT": null,
      "profileBranch": "sphere"
    },
    {
      "name": "outer-tip-crossing",
      "controls": [
        [
          0,
          0.8,
          -0.0008
        ],
        [
          0.3333333333333333,
          1.2,
          -0.0008
        ],
        [
          0.6666666666666666,
          1.2,
          -0.0008
        ],
        [
          1,
          0.8,
          -0.0008
        ]
      ],
      "boundary": [
        [
          [
            -2,
            -1
          ],
          [
            2,
            -1
          ],
          [
            2,
            1
          ],
          [
            -2,
            1
          ]
        ]
      ],
      "expectedSafe": false,
      "witnessT": 0.5,
      "profileBranch": "sphere"
    },
    {
      "name": "outer-body-crossing-tip-inside",
      "controls": [
        [
          0,
          0.85,
          -0.0008
        ],
        [
          0.3333333333333333,
          0.99,
          -0.0008
        ],
        [
          0.6666666666666666,
          0.99,
          -0.0008
        ],
        [
          1,
          0.85,
          -0.0008
        ]
      ],
      "boundary": [
        [
          [
            -2,
            -1
          ],
          [
            2,
            -1
          ],
          [
            2,
            1
          ],
          [
            -2,
            1
          ]
        ]
      ],
      "expectedSafe": false,
      "witnessT": 0.5,
      "profileBranch": "sphere"
    },
    {
      "name": "deep-body-growth-unsafe",
      "controls": [
        [
          0,
          0,
          -0.02
        ],
        [
          0.3333333333333333,
          0,
          -0.22
        ],
        [
          0.6666666666666666,
          0,
          -0.22
        ],
        [
          1,
          0,
          -0.02
        ]
      ],
      "boundary": [
        [
          [
            -2,
            -0.5
          ],
          [
            2,
            -0.5
          ],
          [
            2,
            0.5
          ],
          [
            -2,
            0.5
          ]
        ]
      ],
      "expectedSafe": false,
      "witnessT": 0.5,
      "profileBranch": "sphere"
    },
    {
      "name": "deep-body-growth-safe",
      "controls": [
        [
          0,
          0,
          -0.02
        ],
        [
          0.3333333333333333,
          0,
          -0.03
        ],
        [
          0.6666666666666666,
          0,
          -0.03
        ],
        [
          1,
          0,
          -0.02
        ]
      ],
      "boundary": [
        [
          [
            -2,
            -0.5
          ],
          [
            2,
            -0.5
          ],
          [
            2,
            0.5
          ],
          [
            -2,
            0.5
          ]
        ]
      ],
      "expectedSafe": true,
      "witnessT": null,
      "profileBranch": "sphere"
    },
    {
      "name": "full-profile-deep-safe",
      "controls": [
        [
          0,
          0,
          -12
        ],
        [
          0.3333333333333333,
          0.1,
          -12.7
        ],
        [
          0.6666666666666666,
          0.1,
          -12.7
        ],
        [
          1,
          0,
          -12
        ]
      ],
      "boundary": [
        [
          [
            -5,
            -5
          ],
          [
            5,
            -5
          ],
          [
            5,
            5
          ],
          [
            -5,
            5
          ]
        ]
      ],
      "expectedSafe": true,
      "witnessT": null,
      "profileBranch": "linear"
    },
    {
      "name": "full-profile-deep-unsafe",
      "controls": [
        [
          0,
          0,
          -8
        ],
        [
          0.3333333333333333,
          0,
          -14.266666666666666
        ],
        [
          0.6666666666666666,
          0,
          -14.266666666666666
        ],
        [
          1,
          0,
          -8
        ]
      ],
      "boundary": [
        [
          [
            -10,
            -2.1
          ],
          [
            10,
            -2.1
          ],
          [
            10,
            2.1
          ],
          [
            -10,
            2.1
          ]
        ]
      ],
      "expectedSafe": false,
      "witnessT": 0.5,
      "profileBranch": "linear"
    },
    {
      "name": "above-stock-no-engagement",
      "controls": [
        [
          300,
          0,
          0.1
        ],
        [
          400,
          1,
          0.2
        ],
        [
          450,
          -1,
          0.2
        ],
        [
          500,
          0,
          0.1
        ]
      ],
      "boundary": [
        [
          [
            -2,
            -1
          ],
          [
            2,
            -1
          ],
          [
            2,
            1
          ],
          [
            -2,
            1
          ]
        ]
      ],
      "expectedSafe": true,
      "witnessT": null,
      "profileBranch": "above"
    },
    {
      "name": "surface-crossing-below-stock",
      "controls": [
        [
          0,
          0.8,
          0.1
        ],
        [
          0.3333333333333333,
          0.8,
          -0.1
        ],
        [
          0.6666666666666666,
          0.8,
          -0.1
        ],
        [
          1,
          0.8,
          0.1
        ]
      ],
      "boundary": [
        [
          [
            -2,
            -1
          ],
          [
            2,
            -1
          ],
          [
            2,
            1
          ],
          [
            -2,
            1
          ]
        ]
      ],
      "expectedSafe": false,
      "witnessT": 0.5,
      "profileBranch": "witness"
    },
    {
      "name": "narrow-hole-between-17-poses",
      "controls": [
        [
          0,
          0,
          -0.0008
        ],
        [
          3.3333333333333335,
          2,
          -0.0008
        ],
        [
          6.666666666666667,
          -2,
          -0.0008
        ],
        [
          10,
          0,
          -0.0008
        ]
      ],
      "boundary": [
        [
          [
            -100,
            -100
          ],
          [
            100,
            -100
          ],
          [
            100,
            100
          ],
          [
            -100,
            100
          ]
        ],
        [
          [
            0.3025,
            0.1602880859375
          ],
          [
            0.3025,
            0.1802880859375
          ],
          [
            0.3225,
            0.1802880859375
          ],
          [
            0.3225,
            0.1602880859375
          ]
        ]
      ],
      "expectedSafe": false,
      "witnessT": 0.03125,
      "profileBranch": "hole"
    },
    {
      "name": "outer-body-crossing-tip-inside-reversed",
      "controls": [
        [
          1,
          0.85,
          -0.0008
        ],
        [
          0.6666666666666666,
          0.99,
          -0.0008
        ],
        [
          0.3333333333333333,
          0.99,
          -0.0008
        ],
        [
          0,
          0.85,
          -0.0008
        ]
      ],
      "boundary": [
        [
          [
            -2,
            -1
          ],
          [
            2,
            -1
          ],
          [
            2,
            1
          ],
          [
            -2,
            1
          ]
        ]
      ],
      "expectedSafe": false,
      "witnessT": 0.5,
      "profileBranch": "sphere"
    },
    {
      "name": "outer-body-crossing-tip-inside-translated",
      "controls": [
        [
          300.125,
          418.1,
          -0.0008
        ],
        [
          300.4583333333333,
          418.24,
          -0.0008
        ],
        [
          300.7916666666667,
          418.24,
          -0.0008
        ],
        [
          301.125,
          418.1,
          -0.0008
        ]
      ],
      "boundary": [
        [
          [
            298.125,
            416.25
          ],
          [
            302.125,
            416.25
          ],
          [
            302.125,
            418.25
          ],
          [
            298.125,
            418.25
          ]
        ]
      ],
      "expectedSafe": false,
      "witnessT": 0.5,
      "profileBranch": "sphere"
    },
    {
      "name": "full-profile-deep-safe-reversed",
      "controls": [
        [
          1,
          0,
          -12
        ],
        [
          0.6666666666666666,
          0.1,
          -12.7
        ],
        [
          0.3333333333333333,
          0.1,
          -12.7
        ],
        [
          0,
          0,
          -12
        ]
      ],
      "boundary": [
        [
          [
            -5,
            -5
          ],
          [
            5,
            -5
          ],
          [
            5,
            5
          ],
          [
            -5,
            5
          ]
        ]
      ],
      "expectedSafe": true,
      "witnessT": null,
      "profileBranch": "linear"
    },
    {
      "name": "full-profile-deep-safe-translated",
      "controls": [
        [
          300.125,
          417.25,
          -12
        ],
        [
          300.4583333333333,
          417.35,
          -12.7
        ],
        [
          300.7916666666667,
          417.35,
          -12.7
        ],
        [
          301.125,
          417.25,
          -12
        ]
      ],
      "boundary": [
        [
          [
            295.125,
            412.25
          ],
          [
            305.125,
            412.25
          ],
          [
            305.125,
            422.25
          ],
          [
            295.125,
            422.25
          ]
        ]
      ],
      "expectedSafe": true,
      "witnessT": null,
      "profileBranch": "linear"
    },
    {
      "name": "narrow-hole-between-17-poses-reversed",
      "controls": [
        [
          10,
          0,
          -0.0008
        ],
        [
          6.666666666666667,
          -2,
          -0.0008
        ],
        [
          3.3333333333333335,
          2,
          -0.0008
        ],
        [
          0,
          0,
          -0.0008
        ]
      ],
      "boundary": [
        [
          [
            -100,
            -100
          ],
          [
            100,
            -100
          ],
          [
            100,
            100
          ],
          [
            -100,
            100
          ]
        ],
        [
          [
            0.3025,
            0.1602880859375
          ],
          [
            0.3025,
            0.1802880859375
          ],
          [
            0.3225,
            0.1802880859375
          ],
          [
            0.3225,
            0.1602880859375
          ]
        ]
      ],
      "expectedSafe": false,
      "witnessT": 0.96875,
      "profileBranch": "hole"
    },
    {
      "name": "narrow-hole-between-17-poses-translated",
      "controls": [
        [
          300.125,
          417.25,
          -0.0008
        ],
        [
          303.4583333333333,
          419.25,
          -0.0008
        ],
        [
          306.7916666666667,
          415.25,
          -0.0008
        ],
        [
          310.125,
          417.25,
          -0.0008
        ]
      ],
      "boundary": [
        [
          [
            200.125,
            317.25
          ],
          [
            400.125,
            317.25
          ],
          [
            400.125,
            517.25
          ],
          [
            200.125,
            517.25
          ]
        ],
        [
          [
            300.4275,
            417.4102880859375
          ],
          [
            300.4275,
            417.4302880859375
          ],
          [
            300.4475,
            417.4302880859375
          ],
          [
            300.4475,
            417.4102880859375
          ]
        ]
      ],
      "expectedSafe": false,
      "witnessT": 0.03125,
      "profileBranch": "hole"
    }
  ],
  "coverage": {
    "nativeCurve": [
      [
        0,
        0,
        -1
      ],
      [
        0.29,
        0.377,
        -1
      ],
      [
        0.71,
        0.377,
        -1
      ],
      [
        1,
        0,
        -1
      ]
    ],
    "query": [
      0.5,
      0,
      -0.999
    ],
    "verticalMissingWitness": [
      0.5,
      0,
      -0.977
    ],
    "originalPose": [
      0.5,
      0,
      -1
    ],
    "minimumEuclideanRemovedVolumeGapMm": 0.023,
    "minimumMissingDepthMm": 0.023,
    "existingSurfaceBudgetMm": 0.02,
    "wholeCutterProof": {
      "nose": {
        "certified": true,
        "intervals": 3,
        "method": "exact Fraction Bernstein coefficient/subdivision sign certificate"
      },
      "upperBodyMinimumVerticalSeparationMm": 1.5865,
      "upperBodySeparated": true
    },
    "originalSource": [
      [
        0,
        0,
        -1
      ],
      [
        0.01,
        0.013,
        -1
      ],
      [
        0.03,
        0,
        -1
      ],
      [
        0.97,
        0,
        -1
      ],
      [
        0.99,
        0.013,
        -1
      ],
      [
        1,
        0,
        -1
      ]
    ],
    "proof": {
      "certified": true,
      "intervals": 3,
      "method": "exact Fraction Bernstein coefficient/subdivision sign certificate"
    },
    "meaning": "Original literal source includes the pose at x=.5,y=0,z=-1 and removes query. Exact scalar Bernstein bounds exclude every replacement nose sphere expanded by .023mm from the query; all higher cutter sections are vertically separated by at least 1.5865mm. Thus a point removed by the original has more than .023mm distance from the entire native swept cutter. No prior stock is present in this synthetic case. This is a failure of native removal coverage despite unchanged depth, legal containment, matching endpoint directions and a fit below .3mm."
  }
}
""";
}
